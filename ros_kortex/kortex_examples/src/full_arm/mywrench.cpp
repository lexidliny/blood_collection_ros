/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2019 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */
#include <thread>
#include <atomic>

#include "ros/ros.h"
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/CartesianSpeed.h>
#include <kortex_driver/BaseCyclic_Feedback.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/SetCartesianReferenceFrame.h>
#include <kortex_driver/CartesianReferenceFrame.h>
#include <kortex_driver/SendGripperCommand.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/ActionType.h>
#include <kortex_driver/GetMeasuredCartesianPose.h>
#include <kortex_driver/OnNotificationActionTopic.h>
// #include "kortex_driver/generated/robot/base_ros_converter.h"
#include <kortex_driver/SendWrenchCommand.h>
#include <kortex_driver/StopAction.h>

#define HOME_ACTION_IDENTIFIER 2

bool all_notifs_succeeded = true;

std::atomic<int> last_action_notification_event{0};
std::atomic<int> last_action_notification_id{0};

void notification_callback(const kortex_driver::ActionNotification& notif)
{
  last_action_notification_event = notif.action_event;
  last_action_notification_id = notif.handle.identifier;
}

bool wait_for_action_end_or_abort()
{
  while (ros::ok())
  {
    if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
    {
      ROS_INFO("Received ACTION_END notification for action %d", last_action_notification_id.load());
      return true;
    }
    else if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      ROS_INFO("Received ACTION_ABORT notification for action %d", last_action_notification_id.load());
      all_notifs_succeeded = false;
      return false;
    }
    ros::spinOnce();
  }
  return false;
}

bool example_home_the_robot(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
  kortex_driver::ReadAction service_read_action;

  // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
  service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

  if (!service_client_read_action.call(service_read_action))
  {
    std::string error_string = "Failed to call ReadAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // We can now execute the Action that we read 
  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input = service_read_action.response.output;
  
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("The Home position action was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  return wait_for_action_end_or_abort();
}

bool example_cartesian_action(ros::NodeHandle n, const std::string &robot_name)
{

  // kortex_driver::WrenchCommand wrenchcommand;
  kortex_driver::SendWrenchCommand req;
  req.request.input.wrench.force_x = 0.f;
  req.request.input.wrench.force_y = 5.f;
  req.request.input.wrench.force_z = 0.f;
  req.request.input.wrench.torque_x = 0.f;
  req.request.input.wrench.torque_y = 0.f;
  req.request.input.wrench.torque_z = 0.f;
  req.request.input.duration = 50;
  req.request.input.mode = 0;
  req.request.input.reference_frame =  kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL;

  // wrenchcommand.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL;
  // wrenchcommand.wrench.force_x = 0.f;
  // wrenchcommand.wrench.force_y = 0.f;
  // wrenchcommand.wrench.force_z = 6.f;
  // wrenchcommand.wrench.torque_x = 0.f;
  // wrenchcommand.wrench.torque_y = 0.f;
  // wrenchcommand.wrench.torque_z = 0.f;
  // wrenchcommand.duration = 5000;

  // wrenchcommand.mode = 0;
  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::SendWrenchCommand>("/" + robot_name + "/base/send_wrench_command");
  // ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;
  
  // service_execute_action.request.input.oneof_action_parameters.send_wrench_command.push_back(wrenchcommand);
  service_execute_action.request.input.name = "wrenchcommand";
  service_execute_action.request.input.handle.identifier = 1005;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::SEND_WRENCH_COMMAND;
  
  last_action_notification_event = 0;
  service_client_execute_action.call(req);
//   if (service_client_execute_action.call(req))
//   {
//     ROS_INFO("wrench command was sent to the robot.");
//   }
//   else
//   {
//     std::string error_string = "Failed to call ExecuteAction on pose 5";
//     ROS_ERROR("%s", error_string.c_str());
//     return false;
//   }
//   std::this_thread::sleep_for(std::chrono::milliseconds(5000));

//   // Waiting for the pose 1 to end
//   wait_for_action_end_or_abort();
  std::this_thread::sleep_for(std::chrono::milliseconds(6000));
  wait_for_action_end_or_abort();
  return true;
}

bool stop(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::StopAction stop;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::StopAction>("/" + robot_name + "/base/stop_action");
  // ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");

  
  last_action_notification_event = 0;
  service_client_execute_action.call(stop);
//   if (service_client_execute_action.call(req))
//   {
//     ROS_INFO("wrench command was sent to the robot.");
//   }
//   else
//   {
//     std::string error_string = "Failed to call ExecuteAction on pose 5";
//     ROS_ERROR("%s", error_string.c_str());
//     return false;
//   }
//   std::this_thread::sleep_for(std::chrono::milliseconds(5000));

//   // Waiting for the pose 1 to end
//   wait_for_action_end_or_abort();
  std::this_thread::sleep_for(std::chrono::milliseconds(6000));
  return true;
}

// This function sets the reference frame to the robot's base
bool example_set_cartesian_reference_frame(ros::NodeHandle n, const std::string &robot_name)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call ";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Set reference to base frame.");
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "full_arm_cartesian_action_cpp");

  // For testing purpose
  ros::param::del("/kortex_examples_test_results/cartesian_poses_with_notifications_cpp");

  bool success = true;

  //*******************************************************************************
  // ROS Parameters
  ros::NodeHandle n;
  std::string robot_name = "my_gen3";

  // Parameter robot_name
  if (!ros::param::get("~robot_name", robot_name))
  {
    std::string error_string = "Parameter robot_name was not specified, defaulting to " + robot_name + " as namespace";
    ROS_WARN("%s", error_string.c_str());
  }
  else 
  {
    std::string error_string = "Using robot_name " + robot_name + " as namespace";
    ROS_INFO("%s", error_string.c_str());
  }

  // We need to call this service to activate the Action Notification on the kortex_driver node.
  ros::ServiceClient service_client_activate_notif = n.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + robot_name + "/base/activate_publishing_of_action_topic");
  kortex_driver::OnNotificationActionTopic service_activate_notif;
  if (service_client_activate_notif.call(service_activate_notif))
  {
    ROS_INFO("Action notification activated!");
  }
  else 
  {
    std::string error_string = "Action notification publication failed";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  ros::Duration(1.00).sleep();
  ros::Subscriber sub = n.subscribe("/" + robot_name  + "/action_topic", 1000, notification_callback);

  // Run the example
  // success &= example_set_cartesian_reference_frame(n, robot_name);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  success &= example_home_the_robot(n, robot_name);
  success &= example_cartesian_action(n, robot_name);
  success &= example_home_the_robot(n, robot_name);

  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // // success &= stop(n, robot_name);
  // success &= example_home_the_robot(n, robot_name);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // success &= example_cartesian_action(n, robot_name);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // success &= example_home_the_robot(n, robot_name);
  // success &= all_notifs_succeeded;

  // Report success for testing purposes
  ros::param::set("/kortex_examples_test_results/cartesian_poses_with_notifications_cpp", success);
  
  return success ? 0 : 1;
}