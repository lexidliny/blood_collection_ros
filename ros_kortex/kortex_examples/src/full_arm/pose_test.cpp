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
#include <forehead_detect/GetForeheadPose.h>
#include <tf/tf.h>

#include "dh_gripper_msgs/GripperCtrl.h"
#include "dh_gripper_msgs/GripperState.h"
#include "dh_gripper_msgs/GripperRotCtrl.h"
#include "dh_gripper_msgs/GripperRotState.h"
#include <kortex_driver/SendWrenchCommand.h>
#include <kortex_driver/BaseCyclic_Feedback.h>

#include "hand_mediapipe/FingerTipPose.h"
#include "hand_mediapipe/LandmarkPoint.h"

#include <iostream>
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <argp.h>

ros::Subscriber _sub_grip_state ;
ros::Publisher _gripper_ctrl_pub;
ros::Subscriber _sub_rot_state ;
ros::Publisher _rot_ctrl_pub;

dh_gripper_msgs::GripperState _g_state;
dh_gripper_msgs::GripperRotState _r_state;

#define HOME_ACTION_IDENTIFIER 2

bool all_notifs_succeeded = true;
double target_offset = 0.05;
std::atomic<int> last_action_notification_event{0};
std::atomic<int> last_action_notification_id{0};

void _update_gripper_state(const dh_gripper_msgs::GripperState::ConstPtr& msg)
{
        _g_state.header = msg->header;
        _g_state.is_initialized = msg->is_initialized;
        
        _g_state.grip_state  =  msg->grip_state;
        _g_state.position = msg->position;
        _g_state.target_position = msg->target_position;
        _g_state.target_force = msg->target_force;
         //ROS_INFO("state : %ld %lf %lf %lf %lf", _g_state.is_initialized, _g_state.grip_state, _g_state.position, _g_state.target_position, _g_state.target_force);
}

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

bool set_cartesian_reference_tool_frame(ros::NodeHandle n, const std::string &robot_name)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  else
  {
    ROS_INFO("Set reference to tool frame.");
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  return true;
}

bool set_cartesian_reference_base_frame(ros::NodeHandle n, const std::string &robot_name)
{
  // Initialize the ServiceClient
  ros::ServiceClient service_client_set_cartesian_reference_frame = n.serviceClient<kortex_driver::SetCartesianReferenceFrame>("/" + robot_name + "/control_config/set_cartesian_reference_frame");
  kortex_driver::SetCartesianReferenceFrame service_set_cartesian_reference_frame;

  service_set_cartesian_reference_frame.request.input.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  if (!service_client_set_cartesian_reference_frame.call(service_set_cartesian_reference_frame))
  {
    std::string error_string = "Failed to call SetCartesianReferenceFrame";
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


bool get_needle_up(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;
  my_cartesian_speed.translation = 0.18f;
  my_cartesian_speed.orientation = 40.0f;
  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;
  // std::cout<<"after 2.5 second"<<std::endl;
  sleep(1);

  // Change the pose and give it a new identifier
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.name = "pose1";
  
  my_constrained_pose.target_pose.x = 0.281;
  my_constrained_pose.target_pose.y = -0.259;
  my_constrained_pose.target_pose.z = 0.259;
  
  my_constrained_pose.target_pose.theta_x = 0.;
  my_constrained_pose.target_pose.theta_y = 180.;
  my_constrained_pose.target_pose.theta_z = 0.;

  sleep(1);
  service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();
  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 2 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 2";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  // Waiting for the pose 1 to end
 return wait_for_action_end_or_abort();

}

bool get_needle_down(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;
  my_cartesian_speed.translation = 0.18f;
  my_cartesian_speed.orientation = 40.0f;
  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;
  // std::cout<<"after 2.5 second"<<std::endl;
  sleep(1);

  // Change the pose and give it a new identifier
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.name = "pose1";
  
  my_constrained_pose.target_pose.x = 0.281;
  my_constrained_pose.target_pose.y = -0.259;
  my_constrained_pose.target_pose.z = 0.220;
  
  my_constrained_pose.target_pose.theta_x = 0.;
  my_constrained_pose.target_pose.theta_y = 180.;
  my_constrained_pose.target_pose.theta_z = 0.;

  sleep(1);
  service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();
  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 2 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 2";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  // Waiting for the pose 1 to end
 return wait_for_action_end_or_abort();

}


bool go_up(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;
  my_cartesian_speed.translation = 0.18f;
  my_cartesian_speed.orientation = 40.0f;
  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;
  // std::cout<<"after 2.5 second"<<std::endl;
  sleep(1);

     // Change the pose and give it a new identifier
  service_execute_action.request.input.handle.identifier = 1004;
  service_execute_action.request.input.name = "pose4";
  
  my_constrained_pose.target_pose.x = 0.378;
  my_constrained_pose.target_pose.y = 0.005;
  my_constrained_pose.target_pose.z = 0.501;
  my_constrained_pose.target_pose.theta_x = 180.;
  my_constrained_pose.target_pose.theta_y = 0.;
  my_constrained_pose.target_pose.theta_z = 90.;

  sleep(1);
  service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();
  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 2 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 2";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  std::cout<<"success"<<std::endl;
//   wait_for_action_end_or_abort();


  


  // Waiting for the pose 1 to end
 return wait_for_action_end_or_abort();

}

/* Used by ‘main’ to communicate with ‘parse_opt’. */
bool go_fingertip_up(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;
  my_cartesian_speed.translation = 0.18f;
  my_cartesian_speed.orientation = 40.0f;
  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;
  // std::cout<<"after 2.5 second"<<std::endl;
  sleep(1);
  std::cout<<"after 1 second"<<std::endl;
  ros::ServiceClient getWearingPose = n.serviceClient<hand_mediapipe::FingerTipPose>("/" + robot_name +"/fingertip_pose_server");
  hand_mediapipe::FingerTipPose get;
   std::cout<<"after 1 second"<<std::endl;
  if (getWearingPose.call(get))
  {
    ROS_INFO("get the wearing pose");
    //  Change the pose and give it a new identifier
    service_execute_action.request.input.handle.identifier = 1005;
    service_execute_action.request.input.name = "pose5";
    
    tf::Quaternion q( get.response.pose.orientation.x, get.response.pose.orientation.y,
                      get.response.pose.orientation.z,get.response.pose.orientation.w);
    double roll, pitch, yaw;//x y z
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    roll = roll / M_PI * 180;
    pitch = pitch / M_PI * 180;
    yaw = yaw / M_PI * 180;
    my_constrained_pose.target_pose.x = get.response.pose.position.x;
    my_constrained_pose.target_pose.y = get.response.pose.position.y;
    my_constrained_pose.target_pose.z = get.response.pose.position.z ;
    my_constrained_pose.target_pose.theta_x = roll;
    my_constrained_pose.target_pose.theta_y = pitch;
    my_constrained_pose.target_pose.theta_z = yaw;

    std::cout<<"wearing pose的位置坐标：x="<<my_constrained_pose.target_pose.x<<",y="<<my_constrained_pose.target_pose.y
            <<",z="<<my_constrained_pose.target_pose.z<<std::endl;
        std::cout<<"wearing pose的rotation：theta_x="<<my_constrained_pose.target_pose.theta_x <<",theta_y="<<my_constrained_pose.target_pose.theta_y<<
    ",theta_z="<<my_constrained_pose.target_pose.theta_z<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call get_wearing_pose service");
    return false;
  }

  // my_constrained_pose.target_pose.x = 0;
  // my_constrained_pose.target_pose.y = 0.;
  // my_constrained_pose.target_pose.z = 0.1;
  // my_constrained_pose.target_pose.theta_x = 0;
  // my_constrained_pose.target_pose.theta_y = 0;
  // my_constrained_pose.target_pose.theta_z = 0;


  sleep(1);
  service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();
  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 2 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 2";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  std::cout<<"success"<<std::endl;

  // Waiting for the pose 1 to end
 return wait_for_action_end_or_abort();

}

bool go_palm_up(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;
  my_cartesian_speed.translation = 0.18f;
  my_cartesian_speed.orientation = 40.0f;
  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;
  // std::cout<<"after 2.5 second"<<std::endl;
  sleep(1);
  std::cout<<"after 1 second"<<std::endl;
  ros::ServiceClient getWearingPose = n.serviceClient<hand_mediapipe::FingerTipPose>("/" + robot_name +"/palm_pose_server");
  hand_mediapipe::FingerTipPose get;
   std::cout<<"after 1 second"<<std::endl;
  if (getWearingPose.call(get))
  {
    ROS_INFO("get the wearing pose");
    //  Change the pose and give it a new identifier
    service_execute_action.request.input.handle.identifier = 1005;
    service_execute_action.request.input.name = "pose5";
    
    tf::Quaternion q( get.response.pose.orientation.x, get.response.pose.orientation.y,
                      get.response.pose.orientation.z,get.response.pose.orientation.w);
    double roll, pitch, yaw;//x y z
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    roll = roll / M_PI * 180;
    pitch = pitch / M_PI * 180;
    yaw = yaw / M_PI * 180;
    my_constrained_pose.target_pose.x = get.response.pose.position.x;
    my_constrained_pose.target_pose.y = get.response.pose.position.y;
    my_constrained_pose.target_pose.z = get.response.pose.position.z ;
    my_constrained_pose.target_pose.theta_x = roll;
    my_constrained_pose.target_pose.theta_y = pitch;
    my_constrained_pose.target_pose.theta_z = yaw;

    std::cout<<"wearing pose的位置坐标：x="<<my_constrained_pose.target_pose.x<<",y="<<my_constrained_pose.target_pose.y
            <<",z="<<my_constrained_pose.target_pose.z<<std::endl;
        std::cout<<"wearing pose的rotation：theta_x="<<my_constrained_pose.target_pose.theta_x <<",theta_y="<<my_constrained_pose.target_pose.theta_y<<
    ",theta_z="<<my_constrained_pose.target_pose.theta_z<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call get_wearing_pose service");
    return false;
  }

  // my_constrained_pose.target_pose.x = 0;
  // my_constrained_pose.target_pose.y = 0.;
  // my_constrained_pose.target_pose.z = 0.1;
  // my_constrained_pose.target_pose.theta_x = 0;
  // my_constrained_pose.target_pose.theta_y = 0;
  // my_constrained_pose.target_pose.theta_z = 0;


  sleep(1);
  service_execute_action.request.input.oneof_action_parameters.reach_pose.clear();
  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 2 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 2";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  std::cout<<"success"<<std::endl;

  // Waiting for the pose 1 to end
 return wait_for_action_end_or_abort();

}

bool give_force(ros::NodeHandle n, const std::string &robot_name)
{

  // kortex_driver::WrenchCommand wrenchcommand;
  kortex_driver::SendWrenchCommand req;
  req.request.input.wrench.force_x = 0.f;
  req.request.input.wrench.force_y = 0.f;
  req.request.input.wrench.force_z = 3.f;
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
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  wait_for_action_end_or_abort();
  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "go2head");

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

  // std::string _gripper_model;
  // n.param<std::string>("Gripper_Model", _gripper_model,"PGE");

  // ROS_INFO("Gripper_model : %s", _gripper_model.c_str());
  // _sub_grip_state = n.subscribe("/gripper/states", 50, _update_gripper_state);
  // _gripper_ctrl_pub = n.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/ctrl", 50);

  //  sleep(2);

  // dh_gripper_msgs::GripperCtrl msg_g_ctrl;
  // msg_g_ctrl.initialize = true;
  // msg_g_ctrl.position = 1000;
  // msg_g_ctrl.force = 100;
  // msg_g_ctrl.speed = 100;
  // _gripper_ctrl_pub.publish(msg_g_ctrl);

  // while(!_g_state.is_initialized)
  // {
  //         ros::spinOnce();
  // }

  // sleep(5);
  // msg_g_ctrl.initialize = false;
  // msg_g_ctrl.position = 700;
  // msg_g_ctrl.force = 20;
  // msg_g_ctrl.speed = 20;
  // _gripper_ctrl_pub.publish(msg_g_ctrl);
  // sleep(5);               
  


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
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // success &= go_up(n, robot_name);
  // success &= get_needle_up(n, robot_name);
  // success &= get_needle_down(n, robot_name);

  // msg_g_ctrl.initialize = false;
  // msg_g_ctrl.position = 50;
  // msg_g_ctrl.force = 70;
  // msg_g_ctrl.speed = 20;
  // _gripper_ctrl_pub.publish(msg_g_ctrl);
  // std::cout << "state: " << _g_state.grip_state << std::endl;

  // sleep(5);
  

  // success &= get_needle_up(n, robot_name);
  success &= go_up(n, robot_name);
  // success &= set_cartesian_reference_tool_frame(n, robot_name);
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // sleep(1);
  success &= go_palm_up(n, robot_name);
  sleep(8);
  // success &= give_force(n, robot_name);
//    success &= go_up(n, robot_name);

  // success &= example_cartesian_action(n, robot_name);
  // success &= all_notifs_succeeded;

  // Report success for testing purposes
  ros::param::set("/kortex_examples_test_results/cartesian_poses_with_notifications_cpp", success);
  
  return success ? 0 : 1;
}
