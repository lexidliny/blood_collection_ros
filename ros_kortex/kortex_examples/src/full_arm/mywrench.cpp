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
#include <kortex_driver/Stop.h>
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
#include <kortex_driver/SetPayloadInformation.h>
#include <kortex_driver/GetPayloadInformation.h>

#include <BaseClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <BaseCyclicClientRpc.h>
#include <TransportClientTcp.h>

#include "dh_gripper_msgs/GripperCtrl.h"
#include "dh_gripper_msgs/GripperState.h"
#include "dh_gripper_msgs/GripperRotCtrl.h"
#include "dh_gripper_msgs/GripperRotState.h"

#include <iostream>
#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <argp.h>

namespace k_api = Kinova::Api;

#define PORT 10000
#define HOME_ACTION_IDENTIFIER 2

bool all_notifs_succeeded = true;

std::atomic<int> last_action_notification_event{0};
std::atomic<int> last_action_notification_id{0};

ros::Subscriber _sub_grip_state ;
ros::Publisher _gripper_ctrl_pub;
ros::Subscriber _sub_rot_state ;
ros::Publisher _rot_ctrl_pub;

dh_gripper_msgs::GripperState _g_state;
dh_gripper_msgs::GripperRotState _r_state;

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

bool force(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::WrenchCommand wrenchcommand;
  
  kortex_driver::SendWrenchCommand req;
  req.request.input.wrench.force_x = 0.f;
  req.request.input.wrench.force_y = 0.f;
  req.request.input.wrench.force_z = 2.f;
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
  // service_client_execute_action.call(req);
  if (service_client_execute_action.call(req))
  {
    ROS_INFO("wrench command was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call SendWrenchCommand Server";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  wait_for_action_end_or_abort();
  return true;

    // auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    // auto transport = new k_api::TransportClientTcp();
    // auto router = new k_api::RouterClient(transport, error_callback);
    // transport->connect("192.168.1.10", PORT);

    // // Set session data connection information
    // auto create_session_info = k_api::Session::CreateSessionInfo();
    // create_session_info.set_username("admin");
    // create_session_info.set_password("admin");
    // create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    // create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // // Session manager service wrapper
    // std::cout << "Creating session for communication" << std::endl;
    // auto session_manager = new k_api::SessionManager(router);
    // session_manager->CreateSession(create_session_info);
    // std::cout << "Session created" << std::endl;

    // // Create services
    // auto base = new k_api::Base::BaseClient(router);
    // auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);


    // //=================================================================

    // auto command = k_api::Base::WrenchCommand();
    // command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    // command.set_duration(3000);  // Unlimited time to execute

    // std::cout << "Sending wrench command for 5 seconds..." << std::endl;

    // auto wrench = command.mutable_wrench();
    // wrench->set_force_x(0.0f);
    // wrench->set_force_y(0.0f);
    // wrench->set_force_z(9.0f);
    // wrench->set_torque_x(0.0f);
    // wrench->set_torque_y(0.0f);
    // wrench->set_torque_z(0.0f);
    // base->SendWrenchCommand(command);

    // // Let time for twist to be executed
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // wrench->set_force_x(0.0f);
    // wrench->set_force_y(0.0f);
    // wrench->set_force_z(-10.0f);
    // wrench->set_torque_x(0.0f);
    // wrench->set_torque_y(0.0f);
    // wrench->set_torque_z(0.0f);
    // base->SendWrenchCommand(command);
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // // Make movement stop
    // base->Stop();
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // // =================================================================


    // //  Close API session
    // session_manager->CloseSession();

    // // Deactivate the router and cleanly disconnect from the transport object
    // router->SetActivationStatus(false);
    // transport->disconnect();
    // // sleep(1);
    // // Destroy the API
    // delete base;
    // delete session_manager;
    // delete router;
    // delete transport;

}
bool force2(ros::NodeHandle n, const std::string &robot_name)
{

  // kortex_driver::WrenchCommand wrenchcommand;
  
  kortex_driver::SendWrenchCommand req;
  req.request.input.wrench.force_x = 0.f;
  req.request.input.wrench.force_y = 0.f;
  req.request.input.wrench.force_z = 3.0f;
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

  if (service_client_execute_action.call(req))
  {
    ROS_INFO("wrench command was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call SendWrenchCommand Server";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1800));
  wait_for_action_end_or_abort();
  return true;

    // auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    // auto transport = new k_api::TransportClientTcp();
    // auto router = new k_api::RouterClient(transport, error_callback);
    // transport->connect("192.168.1.10", PORT);

    // // Set session data connection information
    // auto create_session_info = k_api::Session::CreateSessionInfo();
    // create_session_info.set_username("admin");
    // create_session_info.set_password("admin");
    // create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    // create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // // Session manager service wrapper
    // std::cout << "Creating session for communication" << std::endl;
    // auto session_manager = new k_api::SessionManager(router);
    // session_manager->CreateSession(create_session_info);
    // std::cout << "Session created" << std::endl;

    // // Create services
    // auto base = new k_api::Base::BaseClient(router);
    // auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);


    // //=================================================================

    // auto command = k_api::Base::WrenchCommand();
    // command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
    // command.set_duration(3000);  // Unlimited time to execute

    // std::cout << "Sending wrench command for 5 seconds..." << std::endl;

    // auto wrench = command.mutable_wrench();
    // wrench->set_force_x(0.0f);
    // wrench->set_force_y(0.0f);
    // wrench->set_force_z(-5.0f);
    // wrench->set_torque_x(0.0f);
    // wrench->set_torque_y(0.0f);
    // wrench->set_torque_z(0.0f);
    // base->SendWrenchCommand(command);

    // // Let time for twist to be executed
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // // wrench->set_force_x(0.0f);
    // // wrench->set_force_y(0.0f);
    // // wrench->set_force_z(5.0f);
    // // wrench->set_torque_x(0.0f);
    // // wrench->set_torque_y(0.0f);
    // // wrench->set_torque_z(0.0f);
    // // base->SendWrenchCommand(command);
    // // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    // // std::cout << "Stopping robot ..." << std::endl;

    // // Make movement stop
    // // base->Stop();
    // // std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // //=================================================================


    //  // Close API session
    // session_manager->CloseSession();

    // // Deactivate the router and cleanly disconnect from the transport object
    // // router->SetActivationStatus(false);
    // // transport->disconnect();

    // // Destroy the API
    // delete base;
    // delete session_manager;
    // delete router;
    // delete transport;

}





bool twist_control(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::TwistCommand twist;
  twist.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL;
  twist.duration = 2;
  twist.twist.angular_x = 0.;
  twist.twist.angular_y = 0.;
  twist.twist.angular_z = 0.;
  twist.twist.linear_x = 0.;
  twist.twist.linear_y = 0.;
  twist.twist.linear_z = -0.005;


  kortex_driver::TwistCommand twist2;
  twist2.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL;
  twist2.duration = 2;
  twist2.twist.angular_x = 0.;
  twist2.twist.angular_y = 0.;
  twist2.twist.angular_z = 0.;
  twist2.twist.linear_x = 0.;
  twist2.twist.linear_y = 0.;
  twist2.twist.linear_z = 0.;

  ros::Publisher twist_pub = n.advertise<kortex_driver::TwistCommand>("/my_gen3/in/cartesian_velocity", 2);
  ros::Rate loop_rate(100);

  // kortex_driver::BaseFeedback force_feedback;
  
  ros::ServiceClient stop_client = n.serviceClient<kortex_driver::Stop>("/my_gen3/base/stop");
  kortex_driver::Stop stop;
  
  

  
  auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");

  // while(ros::ok)
  // {
  //   twist_pub.publish(twist);
  //   // if(feedback->base.tool_external_wrench_force_z > 100)
  //   // {
  //   //   // twist_pub.publish(twist2);
  //   //   std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  //   // }
  //   std::cout << "force: " << feedback->base.tool_external_wrench_force_z << endl;
  //   std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  // }
  twist_pub.publish(twist);
    // if(feedback->base.tool_external_wrench_force_z > 100)
    // {
    //   // twist_pub.publish(twist2);
    //   std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // }
    std::cout << "force: " << feedback->base.tool_external_wrench_force_z << endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    // if (stop_client.call(stop))
    // {
    //   ROS_INFO("Get payload information from the robot.");
    //  ;
    // }
    // else
    // {
    // std::string error_string = "Failed to call GetPayloadInformation Service";
    // ROS_ERROR("%s", error_string.c_str());
    
    // }
    twist_pub.publish(twist2);
    // twist_pub.publish(twist2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    




}


bool set_payload(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::SetPayloadInformation req;
  kortex_driver::ControlConfig_Position position;
  position.x = 0.;
  position.y = 0.;
  position.z = 0.035;
  
  req.request.input.payload_mass = 1.26;
  req.request.input.payload_mass_center = position;
 
  ros::ServiceClient service_client_config = n.serviceClient<kortex_driver::SetPayloadInformation>("/" + robot_name + "/control_config/set_payload_information");
  if (service_client_config.call(req))
  {
    ROS_INFO("SetPayloadInformation command was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 5";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  std::cout << "set payload done" << std::endl;
  return true;
}

bool get_payload(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::GetPayloadInformation req;
  ros::ServiceClient service_client_config = n.serviceClient<kortex_driver::GetPayloadInformation>("/" + robot_name + "/control_config/get_payload_information");
  if (service_client_config.call(req))
  {
    ROS_INFO("Get payload information from the robot.");
    std::cout << "mass: " << req.response.output.payload_mass << std::endl;
  }
  else
  {
    std::string error_string = "Failed to call GetPayloadInformation Service";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

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


bool go_up(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.4f;
  my_cartesian_speed.orientation = 40.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = 0.450;
  my_constrained_pose.target_pose.y = 0.018;
  my_constrained_pose.target_pose.z = 0.260;
  // my_constrained_pose.target_pose.x = 0.552;
  // my_constrained_pose.target_pose.y = -0.198;
  // my_constrained_pose.target_pose.z = 0.194;
  my_constrained_pose.target_pose.theta_x = 180.;
  my_constrained_pose.target_pose.theta_y = 0.;
  my_constrained_pose.target_pose.theta_z = 90.;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose2";
  service_execute_action.request.input.handle.identifier = 1002;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
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
  wait_for_action_end_or_abort();


  return true;
}

bool go_start(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.2f;
  my_cartesian_speed.orientation = 50.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = 0.450;
  my_constrained_pose.target_pose.y = 0.018;
  my_constrained_pose.target_pose.z = 0.380;
  // my_constrained_pose.target_pose.x = 0.552;
  // my_constrained_pose.target_pose.y = -0.198;
  // my_constrained_pose.target_pose.z = 0.194;
  my_constrained_pose.target_pose.theta_x = 180.;
  my_constrained_pose.target_pose.theta_y = 0.;
  my_constrained_pose.target_pose.theta_z = 90.;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose1";
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 1 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 1 to end
  wait_for_action_end_or_abort();


  return true;
}

bool go_start2(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.3f;
  my_cartesian_speed.orientation = 50.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = 0.450;
  my_constrained_pose.target_pose.y = 0.018;
  my_constrained_pose.target_pose.z = 0.380;
  // my_constrained_pose.target_pose.x = 0.552;
  // my_constrained_pose.target_pose.y = -0.198;
  // my_constrained_pose.target_pose.z = 0.194;
  my_constrained_pose.target_pose.theta_x = 180.;
  my_constrained_pose.target_pose.theta_y = 0.;
  my_constrained_pose.target_pose.theta_z = 0.;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose1";
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 1 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 1 to end
  wait_for_action_end_or_abort();


  return true;
}

bool example_set_cartesian_reference_frame(ros::NodeHandle n, const std::string &robot_name)
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


bool setWrenchThroughAPI()
{
  // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect("192.168.1.10", PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);

     // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete session_manager;
    delete router;
    delete transport;

    // return success ? 0: 1;
}

bool go_lancet_up(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.2f;
  my_cartesian_speed.orientation = 70.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = 0.376;
  my_constrained_pose.target_pose.y = -0.327;
  my_constrained_pose.target_pose.z = 0.256;
  // my_constrained_pose.target_pose.x = 0.552;
  // my_constrained_pose.target_pose.y = -0.198;
  // my_constrained_pose.target_pose.z = 0.194;
  my_constrained_pose.target_pose.theta_x = 0.;
  my_constrained_pose.target_pose.theta_y = -180.;
  my_constrained_pose.target_pose.theta_z = 0.;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose1";
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 1 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 1 to end
  wait_for_action_end_or_abort();


  return true;
}

bool go_lancet(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.02f;
  my_cartesian_speed.orientation = 30.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

   my_constrained_pose.target_pose.x = 0.376;
  my_constrained_pose.target_pose.y = -0.327;
  my_constrained_pose.target_pose.z = 0.226;
  // my_constrained_pose.target_pose.x = 0.552;
  // my_constrained_pose.target_pose.y = -0.198;
  // my_constrained_pose.target_pose.z = 0.194;
  my_constrained_pose.target_pose.theta_x = 0.;
  my_constrained_pose.target_pose.theta_y = -180.;
  my_constrained_pose.target_pose.theta_z = 0.;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose1";
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 1 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 1 to end
  wait_for_action_end_or_abort();


  return true;
}

bool go_sensor_up(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.2f;
  my_cartesian_speed.orientation = 60.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = 0.409;
  my_constrained_pose.target_pose.y = -0.279;
  my_constrained_pose.target_pose.z = 0.347;
  // my_constrained_pose.target_pose.x = 0.552;
  // my_constrained_pose.target_pose.y = -0.198;
  // my_constrained_pose.target_pose.z = 0.194;
  my_constrained_pose.target_pose.theta_x = 0.;
  my_constrained_pose.target_pose.theta_y = 180.;
  my_constrained_pose.target_pose.theta_z = 90.;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose1";
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 1 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 1 to end
  wait_for_action_end_or_abort();


  return true;
}

bool go_sensor_up_later(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.1f;
  my_cartesian_speed.orientation = 30.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = 0.409;
  my_constrained_pose.target_pose.y = -0.279;
  my_constrained_pose.target_pose.z = 0.360;
  // my_constrained_pose.target_pose.x = 0.552;
  // my_constrained_pose.target_pose.y = -0.198;
  // my_constrained_pose.target_pose.z = 0.194;
  my_constrained_pose.target_pose.theta_x = 0.;
  my_constrained_pose.target_pose.theta_y = 180.;
  my_constrained_pose.target_pose.theta_z = 90.;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;

  service_execute_action.request.input.oneof_action_parameters.reach_pose.push_back(my_constrained_pose);
  service_execute_action.request.input.name = "pose1";
  service_execute_action.request.input.handle.identifier = 1001;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::REACH_POSE;
  
  last_action_notification_event = 0;
  if (service_client_execute_action.call(service_execute_action))
  {
    ROS_INFO("Pose 1 was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call ExecuteAction on pose 1";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Waiting for the pose 1 to end
  wait_for_action_end_or_abort();


  return true;
}


bool force_sensor(ros::NodeHandle n, const std::string &robot_name)
{

  
  kortex_driver::SendWrenchCommand req;
  req.request.input.wrench.force_x = 0.f;
  req.request.input.wrench.force_y = 0.f;
  req.request.input.wrench.force_z = 5.f;
  req.request.input.wrench.torque_x = 0.f;
  req.request.input.wrench.torque_y = 0.f;
  req.request.input.wrench.torque_z = 0.f;
  req.request.input.duration = 50;
  req.request.input.mode = 0;
  req.request.input.reference_frame =  kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL;

  // wrenchcommand.mode = 0;
  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::SendWrenchCommand>("/" + robot_name + "/base/send_wrench_command");
  // ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");dd

  
  last_action_notification_event = 0;

  if (service_client_execute_action.call(req))
  {
    ROS_INFO("wrench command was sent to the robot.");
  }
  else
  {
    std::string error_string = "Failed to call SendWrenchCommand Server";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  wait_for_action_end_or_abort();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "full_arm_cartesian_action_cpp");

  // For testing purpose
  // ros::param::del("/kortex_examples_test_results/cartesian_poses_with_notifications_cpp");

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

  //=========================================================================

  std::string _gripper_model;
  n.param<std::string>("Gripper_Model", _gripper_model,"PGE");

  ROS_INFO("Gripper_model : %s", _gripper_model.c_str());
  _sub_grip_state = n.subscribe("/gripper/states", 50, _update_gripper_state);
  _gripper_ctrl_pub = n.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/ctrl", 50);

  //  sleep(2);

  dh_gripper_msgs::GripperCtrl msg_g_ctrl;
  msg_g_ctrl.initialize = true;
  msg_g_ctrl.position = 1000;
  msg_g_ctrl.force = 100;
  msg_g_ctrl.speed = 100;
  _gripper_ctrl_pub.publish(msg_g_ctrl);

  while(!_g_state.is_initialized)
  {
          ros::spinOnce();
  }

  sleep(5);
  msg_g_ctrl.initialize = false;
  msg_g_ctrl.position = 700;
  msg_g_ctrl.force = 20;
  msg_g_ctrl.speed = 20;
  _gripper_ctrl_pub.publish(msg_g_ctrl);
  sleep(2);    




  // Run the exampl
  // success &= example_set_cartesian_reference_frame(n, robot_name);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  success &= set_payload(n, robot_name);
  // // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // success &= get_payload(n, robot_name);

  // success &= go_start(n, robot_name);
  // success &= go_start2(n, robot_name);

  success &= go_sensor_up(n, robot_name);
  // success &= force_sensor(n, robot_name);
  // sleep(2);
  // success &= go_sensor_up_later(n, robot_name);





  success &= go_lancet_up(n, robot_name);
  success &= go_lancet(n, robot_name);

  // msg_g_ctrl.initialize = false;
  // msg_g_ctrl.position = 100;
  // msg_g_ctrl.force = 50;
  // msg_g_ctrl.speed = 20;
  // _gripper_ctrl_pub.publish(msg_g_ctrl);
  //   sleep(2);   

  // success &= go_lancet_up(n, robot_name);

  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // setWrenchThroughAPI();
  // success &= example_home_the_robot(n, robot_name);
  // success &= go_start(n, robot_name);
  // // twist_control(n, robot_name);
  // success &= go_up(n, robot_name);
  // sleep(3);
  // success &= force(n, robot_name);
  // success &= force2(n, robot_name);
  // success &= example_set_cartesian_reference_frame(n, robot_name);

  // sleep(1);
  // success &= example_home_the_robot(n, robot_name);
  // success &= go_start(n, robot_name);
  // success &= go_up(n, robot_name);
  // sleep(1);
  // success &= go_start(n, robot_name);
  // success &= go_up(n, robot_name);
  // success &= go_start(n, robot_name);
  // success &= example_home_the_robot(n, robot_name);

  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // // success &= stop(n, robot_name);
  // success &= example_home_the_robot(n, robot_name);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // success &= example_cartesian_action(n, robot_name);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // success &= example_home_the_robot(n, robot_name);
  // success &= all_notifs_succeeded;

  // Report success for testing purposes
  // ros::param::set("/kortex_examples_test_results/cartesian_poses_with_notifications_cpp", success);
  
  return success ? 0 : 1;
}