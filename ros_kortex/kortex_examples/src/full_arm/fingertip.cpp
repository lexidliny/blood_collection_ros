#include <thread>
#include <atomic>

#include "actionlib/client/simple_action_client.h"
#include <angles/angles.h>
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
#include "kortex_driver/FollowCartesianTrajectoryAction.h"
#include "kortex_driver/FollowCartesianTrajectoryActionFeedback.h"
#include "kortex_driver/FollowCartesianTrajectoryActionGoal.h"
#include "kortex_driver/FollowCartesianTrajectoryActionResult.h"
#include <kortex_driver/GetProductConfiguration.h>
#include <kortex_driver/ModelId.h>
#include <kortex_driver/SetPayloadInformation.h>
#include <kortex_driver/GetPayloadInformation.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <kortex_driver/SendWrenchCommand.h>
#include <kortex_driver/StopAction.h>

#include "hand_mediapipe/FingerTipPose.h"
#include "hand_mediapipe/LandmarkPoint.h"

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


#define HOME_ACTION_IDENTIFIER 2

tf::StampedTransform lancet2root;
std::vector<double> lancet_fetch_pose = {0.376, -0.327, 0.224, 0., -180., 0.};
std::vector<double> collector_fetch_pose = {0.439, -0.327,0.251, 0., -180., 90.};
std::vector<double> put_lancet_pose = {0.03, -0.336, 0.33, -180.,0.,180.};

std::vector<double> put_sensor_pose = {0.066, -0.463, 0.219, 0.,180.,0.};

bool all_notifs_succeeded = true;

std::atomic<int> last_action_notification_event{0};
std::atomic<int> last_action_notification_id{0};

std::vector<double> fingertip_pose;
std::vector<double> collector_pose;

ros::Subscriber _sub_grip_state ;
ros::Publisher _gripper_ctrl_pub;
dh_gripper_msgs::GripperState _g_state;
dh_gripper_msgs::GripperCtrl msg_g_ctrl;


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

void grip_init(ros::NodeHandle n)
{
  std::string _gripper_model;
  n.param<std::string>("Gripper_Model", _gripper_model,"PGE");

  ROS_INFO("Gripper_model : %s", _gripper_model.c_str());
  _sub_grip_state = n.subscribe("/gripper/states", 50, _update_gripper_state);
  _gripper_ctrl_pub = n.advertise<dh_gripper_msgs::GripperCtrl>("/gripper/ctrl", 50);

  sleep(1);
  msg_g_ctrl.initialize = true;
  msg_g_ctrl.position = 1000;
  msg_g_ctrl.force = 100;
  msg_g_ctrl.speed = 100;
  _gripper_ctrl_pub.publish(msg_g_ctrl);

  while(!_g_state.is_initialized)
  {
          ros::spinOnce();
  }
  //=================================================================
  sleep(1);
}

void grip_open()
{
  msg_g_ctrl.initialize = false;
  msg_g_ctrl.position = 700;
  msg_g_ctrl.force = 20;
  msg_g_ctrl.speed = 20;
  _gripper_ctrl_pub.publish(msg_g_ctrl);
  sleep(1);  
}

void grip_close()
{
  msg_g_ctrl.initialize = false;
  msg_g_ctrl.position = 20;
  msg_g_ctrl.force = 50;
  msg_g_ctrl.speed = 20;
  _gripper_ctrl_pub.publish(msg_g_ctrl);
  sleep(1);  
}

void notification_callback(const kortex_driver::ActionNotification& notif)
{
  last_action_notification_event = notif.action_event;
}

bool wait_for_action_end_or_abort()
{
  while (ros::ok())
  {
    if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END)
    {
      ROS_INFO("Received ACTION_END notification");
      return true;
    }
    else if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT)
    {
      ROS_INFO("Received ACTION_ABORT notification");
      return false;
    }
    ros::spinOnce();
  }
  return false;
}

bool example_clear_faults(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_clear_faults = n.serviceClient<kortex_driver::Base_ClearFaults>("/" + robot_name + "/base/clear_faults");
  kortex_driver::Base_ClearFaults service_clear_faults;

  // Clear the faults
  if (!service_client_clear_faults.call(service_clear_faults))
  {
    std::string error_string = "Failed to clear the faults";
    ROS_ERROR("%s", error_string.c_str());
    return false;
  }

  // Wait a bit
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  return true;
}


std::vector<double> getpushbeampose(ros::NodeHandle n)
{
	tf::TransformListener listener;
    std::cout<<"1. lancet 和 pushbeam  frame相通"<<std::endl;
    listener.waitForTransform("base_link","lancet",ros::Time(0),ros::Duration(4.0));
    
    try{
     //2. 监听对应的tf,返回平移和旋转
    std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
    listener.lookupTransform("/base_link", "/lancet",
        ros::Time(0), lancet2root);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
	}

	tf::StampedTransform pushbeam2ee;
    listener.waitForTransform("new_end_effector_link","pushbeam",ros::Time(0),ros::Duration(4.0));
    try{
            //2. 监听对应的tf,返回平移和旋转
            std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
            listener.lookupTransform("/new_end_effector_link", "/pushbeam",
            ros::Time(0), pushbeam2ee);
        }
    catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
    }
    
	tf::Transform new_ee2root;
	// new_ee2root.mult(pushbeam2ee.inverse(), lancet2root);
	new_ee2root.mult(lancet2root, pushbeam2ee.inverse());
	// std::cout << new_ee2root <<  std::endl;

	std::vector<double> pose;
	pose.push_back(new_ee2root.getOrigin().x());
	pose.push_back(new_ee2root.getOrigin().y());
	pose.push_back(new_ee2root.getOrigin().z());

	double roll, pitch, yaw;
	tf::Matrix3x3(new_ee2root.getRotation()).getRPY(roll, pitch, yaw);

	pose.push_back(roll / M_PI * 180.);
	pose.push_back(pitch / M_PI * 180.);
	pose.push_back(yaw / M_PI * 180.);

	return pose;

}

std::vector<double> getzhixuepose(ros::NodeHandle n)
{
	tf::TransformListener listener;
    
	tf::StampedTransform zhixue2ee;
    listener.waitForTransform("new_end_effector_link","zhixue",ros::Time(0),ros::Duration(4.0));
    try{
            //2. 监听对应的tf,返回平移和旋转
            std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
            listener.lookupTransform("/new_end_effector_link", "/zhixue",
            ros::Time(0), zhixue2ee);
        }
    catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
    }
    
	tf::Transform new_ee2root;
	// new_ee2root.mult(pushbeam2ee.inverse(), lancet2root);
	new_ee2root.mult(lancet2root, zhixue2ee.inverse());
	// std::cout << new_ee2root <<  std::endl;

	std::vector<double> pose;
	pose.push_back(new_ee2root.getOrigin().x());
	pose.push_back(new_ee2root.getOrigin().y());
	pose.push_back(new_ee2root.getOrigin().z());

	double roll, pitch, yaw;
	tf::Matrix3x3(new_ee2root.getRotation()).getRPY(roll, pitch, yaw);

	pose.push_back(roll / M_PI * 180.);
	pose.push_back(pitch / M_PI * 180.);
	pose.push_back(yaw / M_PI * 180.);

	return pose;

}

void getcollectorpose(ros::NodeHandle n)
{
	tf::TransformListener listener;
    std::cout<<"1. lancet 和 pushbeam  frame相通"<<std::endl;
    listener.waitForTransform("base_link","collector",ros::Time(0),ros::Duration(4.0));
    // from root to collector
    tf::StampedTransform collector2root;
    try{
     //2. 监听对应的tf,返回平移和旋转
    std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
    listener.lookupTransform("/base_link", "/collector",
        ros::Time(0), collector2root);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
	}

	tf::StampedTransform collector2ee;
    listener.waitForTransform("new_end_effector_link","collector",ros::Time(0),ros::Duration(4.0));
    try{
            //2. 监听对应的tf,返回平移和旋转
            std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
            listener.lookupTransform("/new_end_effector_link", "/collector",
            ros::Time(0), collector2ee);
        }
    catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
    }
    
	tf::Transform new_ee2root;
	// new_ee2root.mult(pushbeam2ee.inverse(), lancet2root);
	new_ee2root.mult(lancet2root, collector2ee.inverse());
	// std::cout << new_ee2root <<  std::endl;

	std::vector<double> pose;
	collector_pose.push_back(new_ee2root.getOrigin().x());
	collector_pose.push_back(new_ee2root.getOrigin().y());
	collector_pose.push_back(new_ee2root.getOrigin().z());

	double roll, pitch, yaw;
	tf::Matrix3x3(new_ee2root.getRotation()).getRPY(roll, pitch, yaw);

	collector_pose.push_back(roll / M_PI * 180.);
	collector_pose.push_back(pitch / M_PI * 180.);
	collector_pose.push_back(yaw / M_PI * 180.);

	// return pose;

}

std::vector<double> getdisinfectorpose(ros::NodeHandle n, tf::Quaternion q) 
{
	tf::TransformListener listener;
    
	tf::StampedTransform disinfector2ee;
    listener.waitForTransform("new_end_effector_link","disinfector",ros::Time(0),ros::Duration(4.0));
    try{
            //2. 监听对应的tf,返回平移和旋转
            std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
            listener.lookupTransform("/new_end_effector_link", "/disinfector",
            ros::Time(0), disinfector2ee);
        }
    catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
    }

    tf::StampedTransform lancet2ee;
    listener.waitForTransform("new_end_effector_link","lancet",ros::Time(0),ros::Duration(4.0));
    try{
            //2. 监听对应的tf,返回平移和旋转
            std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
            listener.lookupTransform("/new_end_effector_link", "/lancet",
            ros::Time(0), lancet2ee);
        }
    catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
    }
    
	tf::Transform new_ee2root;
	
  tf::Transform lancent_ee2root;
  tf::Transform fingertip2root;
  lancent_ee2root.setOrigin({fingertip_pose[0],fingertip_pose[1],fingertip_pose[2]});
  
  lancent_ee2root.setRotation(q);
  fingertip2root.mult(lancent_ee2root, lancet2ee);
	new_ee2root.mult(fingertip2root, disinfector2ee.inverse());
	
	std::vector<double> pose;
	pose.push_back(new_ee2root.getOrigin().x());
	pose.push_back(new_ee2root.getOrigin().y());
	pose.push_back(new_ee2root.getOrigin().z());

	double roll, pitch, yaw;
	tf::Matrix3x3(new_ee2root.getRotation()).getRPY(roll, pitch, yaw);

	pose.push_back(roll / M_PI * 180.);
	pose.push_back(pitch / M_PI * 180.);
	pose.push_back(yaw / M_PI * 180.);

  return pose;
}


bool example_home_the_robot(ros::NodeHandle n, const std::string &robot_name)
{
  ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
  kortex_driver::ReadAction service_read_action;
  last_action_notification_event = 0;

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

bool example_cartesian_waypoint_action(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = 0.450;
  cartesianWaypoint.pose.y = 0.018;
  cartesianWaypoint.pose.z = 0.380;
  cartesianWaypoint.pose.theta_x = angles::from_degrees(180.);
  cartesianWaypoint.pose.theta_y = 0;
  cartesianWaypoint.pose.theta_z = angles::from_degrees(90.);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 70.;
  cartesianWaypoint.maximum_linear_velocity = 0.08;

  goal.trajectory.push_back(cartesianWaypoint);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}


bool collector_up(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = collector_pose[0];
  cartesianWaypoint.pose.y = collector_pose[1];
  cartesianWaypoint.pose.z = collector_pose[2];
  cartesianWaypoint.pose.theta_x = angles::from_degrees(collector_pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(collector_pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(collector_pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 40.;
  cartesianWaypoint.maximum_linear_velocity = 0.1;

  goal.trajectory.push_back(cartesianWaypoint);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}

bool pushbeam_up(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  std::vector<double> pose;
  pose = getpushbeampose(n);
  getcollectorpose(n);

  cartesianWaypoint.pose.x = pose[0];
  cartesianWaypoint.pose.y = pose[1];
  cartesianWaypoint.pose.z = pose[2] - 0.015;
  cartesianWaypoint.pose.theta_x = angles::from_degrees(pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 30.;
  cartesianWaypoint.maximum_linear_velocity = 0.06;

  goal.trajectory.push_back(cartesianWaypoint);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}


bool after_push_up(ros::NodeHandle n, const std::string &robot_name)
{

  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

   auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");
  // Initialize the ServiceClient

  // Initialize input
  double current_x = feedback->base.commanded_tool_pose_x ;
  double current_y = feedback->base.commanded_tool_pose_y ;
  double current_z = feedback->base.commanded_tool_pose_z ;
  double current_theta_x = feedback->base.commanded_tool_pose_theta_x;
  double current_theta_y = feedback->base.commanded_tool_pose_theta_y;
  double current_theta_z = feedback->base.commanded_tool_pose_theta_z;

  cartesianWaypoint.pose.x = current_x;
  cartesianWaypoint.pose.y = current_y;
  cartesianWaypoint.pose.z = current_z + 0.1;
  cartesianWaypoint.pose.theta_x = angles::from_degrees(current_theta_x);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(current_theta_y);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(current_theta_z);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 30.;
  cartesianWaypoint.maximum_linear_velocity = 0.06;

  goal.trajectory.push_back(cartesianWaypoint);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
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

bool force(ros::NodeHandle n, const std::string &robot_name)
{

  // kortex_driver::WrenchCommand wrenchcommand;
  kortex_driver::SendWrenchCommand req;
  req.request.input.wrench.force_x = 0.f;
  req.request.input.wrench.force_y = 0.f;
  req.request.input.wrench.force_z = 9.f;
  req.request.input.wrench.torque_x = 0.f;
  req.request.input.wrench.torque_y = 0.f;
  req.request.input.wrench.torque_z = 0.f;
  req.request.input.duration = 50;
  req.request.input.mode = 0;
  req.request.input.reference_frame =  kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL;

  ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::SendWrenchCommand>("/" + robot_name + "/base/send_wrench_command");
  // ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
  kortex_driver::ExecuteAction service_execute_action;
  
  // service_execute_action.request.input.oneof_action_parameters.send_wrench_command.push_back(wrenchcommand);
  service_execute_action.request.input.name = "wrenchcommand";
  service_execute_action.request.input.handle.identifier = 1005;
  service_execute_action.request.input.handle.action_type = kortex_driver::ActionType::SEND_WRENCH_COMMAND;
  
  last_action_notification_event = 0;
  service_client_execute_action.call(req);

  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  wait_for_action_end_or_abort();
  return true;
}

bool go_up(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.2f;
  my_cartesian_speed.orientation = 50.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);


 my_constrained_pose.target_pose.x = 0.378;
  my_constrained_pose.target_pose.y = 0.005;
  my_constrained_pose.target_pose.z = 0.45;

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

bool go_fingertip_up(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = fingertip_pose[0];
  cartesianWaypoint.pose.y = fingertip_pose[1];
  cartesianWaypoint.pose.z = fingertip_pose[2];
  cartesianWaypoint.pose.theta_x = angles::from_degrees(fingertip_pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(fingertip_pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(fingertip_pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 30.;
  cartesianWaypoint.maximum_linear_velocity = 0.04;

  goal.trajectory.push_back(cartesianWaypoint);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}

bool go_disinfector_up(ros::NodeHandle n, const std::string &robot_name)
{

  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;


  //********************************
  ros::ServiceClient getWearingPose = n.serviceClient<hand_mediapipe::FingerTipPose>("/" + robot_name +"/fingertip_pose_server");
  hand_mediapipe::FingerTipPose get;
  std::cout<<"get fingertip pose and disinfector pose"<<std::endl;
  if (getWearingPose.call(get))
  {
    ROS_INFO("get the disinfector pose");
    
    tf::Quaternion q( get.response.pose.orientation.x, get.response.pose.orientation.y,
                      get.response.pose.orientation.z,get.response.pose.orientation.w);
    double roll, pitch, yaw;//x y z
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    roll = roll / M_PI * 180.;
    pitch = pitch / M_PI * 180.;
    yaw = yaw / M_PI * 180.;
    fingertip_pose.push_back(get.response.pose.position.x);
    fingertip_pose.push_back(get.response.pose.position.y);
    fingertip_pose.push_back(get.response.pose.position.z);
    fingertip_pose.push_back(roll);
    fingertip_pose.push_back(pitch);
    fingertip_pose.push_back(yaw);

    std::vector<double> disinfectoruppose = getdisinfectorpose(n, q);

    kortex_driver::CartesianWaypoint cartesianWaypoint;

    cartesianWaypoint.pose.x = disinfectoruppose[0];
    cartesianWaypoint.pose.y = disinfectoruppose[1];
    cartesianWaypoint.pose.z = disinfectoruppose[2] + 0.05;
    cartesianWaypoint.pose.theta_x = angles::from_degrees(disinfectoruppose[3]);
    cartesianWaypoint.pose.theta_y = angles::from_degrees(disinfectoruppose[4]);
    cartesianWaypoint.pose.theta_z = angles::from_degrees(disinfectoruppose[5]);
    cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
    cartesianWaypoint.blending_radius = 0;
    cartesianWaypoint.maximum_angular_velocity = 30.;
    cartesianWaypoint.maximum_linear_velocity = 0.04;

    goal.trajectory.push_back(cartesianWaypoint);
    for(auto i:disinfectoruppose)
    {
      std::cout << " disinfector pose: "<< i << ", " << std::endl;
    }
  }
  else
  {
    ROS_ERROR("Failed to call get_wearing_pose service");
    return false;
  }

  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;

}


bool go_fingertip_up_constrained_pose(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.20f;
  my_cartesian_speed.orientation = 50.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);


  my_constrained_pose.target_pose.x = fingertip_pose[0];
  my_constrained_pose.target_pose.y = fingertip_pose[1];
  my_constrained_pose.target_pose.z = fingertip_pose[2];
  my_constrained_pose.target_pose.theta_x = fingertip_pose[3];
  my_constrained_pose.target_pose.theta_y = fingertip_pose[4];
  my_constrained_pose.target_pose.theta_z = fingertip_pose[5];

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


bool force2(ros::NodeHandle n, const std::string &robot_name)
{

  
  kortex_driver::SendWrenchCommand req;
  req.request.input.wrench.force_x = 0.f;
  req.request.input.wrench.force_y = 0.f;
  req.request.input.wrench.force_z = 4.05;
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

  // std::this_thread::sleep_for(std::chrono::milliseconds(8000));
  wait_for_action_end_or_abort();
  return true;
}

bool go_lancet(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = lancet_fetch_pose[0];
  cartesianWaypoint.pose.y = lancet_fetch_pose[1];
  cartesianWaypoint.pose.z = lancet_fetch_pose[2];
  cartesianWaypoint.pose.theta_x = angles::from_degrees(lancet_fetch_pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(lancet_fetch_pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(lancet_fetch_pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 30.;
  cartesianWaypoint.maximum_linear_velocity = 0.02;

  goal.trajectory.push_back(cartesianWaypoint);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}

bool go_lancet_up(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = lancet_fetch_pose[0];
  cartesianWaypoint.pose.y = lancet_fetch_pose[1];
  cartesianWaypoint.pose.z = lancet_fetch_pose[2] + 0.06;
  cartesianWaypoint.pose.theta_x = angles::from_degrees(lancet_fetch_pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(lancet_fetch_pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(lancet_fetch_pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 30.;
  cartesianWaypoint.maximum_linear_velocity = 0.06;

  goal.trajectory.push_back(cartesianWaypoint);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}

bool go_lancet_up_and_go_up(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;


  cartesianWaypoint.pose.x = lancet_fetch_pose[0];
  cartesianWaypoint.pose.y = lancet_fetch_pose[1];
  cartesianWaypoint.pose.z = lancet_fetch_pose[2] + 0.06;
  cartesianWaypoint.pose.theta_x = angles::from_degrees(lancet_fetch_pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(lancet_fetch_pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(lancet_fetch_pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 30.;
  cartesianWaypoint.maximum_linear_velocity = 0.06;

  kortex_driver::CartesianWaypoint cartesianWaypoint1;

  cartesianWaypoint1.pose.x = 0.378;
  cartesianWaypoint1.pose.y = 0.005;
  cartesianWaypoint1.pose.z = 0.45;
  cartesianWaypoint1.pose.theta_x = angles::from_degrees(180.);
  cartesianWaypoint1.pose.theta_y = angles::from_degrees(0.);
  cartesianWaypoint1.pose.theta_z = angles::from_degrees(90.);
  cartesianWaypoint1.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint1.blending_radius = 0;
  cartesianWaypoint1.maximum_angular_velocity = 45.;
  cartesianWaypoint1.maximum_linear_velocity = 0.1;

  goal.trajectory.push_back(cartesianWaypoint);
  goal.trajectory.push_back(cartesianWaypoint1);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}

bool put_lancet(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = put_lancet_pose[0];
  cartesianWaypoint.pose.y = put_lancet_pose[1];
  cartesianWaypoint.pose.z = put_lancet_pose[2];
  cartesianWaypoint.pose.theta_x = angles::from_degrees(put_lancet_pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(put_lancet_pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(put_lancet_pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 40.;
  cartesianWaypoint.maximum_linear_velocity = 0.1;

  goal.trajectory.push_back(cartesianWaypoint);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}

bool go_sensor_up(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = collector_fetch_pose[0];
  cartesianWaypoint.pose.y = collector_fetch_pose[1];
  cartesianWaypoint.pose.z = collector_fetch_pose[2];
  cartesianWaypoint.pose.theta_x = angles::from_degrees(collector_fetch_pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(collector_fetch_pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(collector_fetch_pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 30.;
  cartesianWaypoint.maximum_linear_velocity = 0.1;

  goal.trajectory.push_back(cartesianWaypoint);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;

}

bool go_sensor_up_later(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  my_cartesian_speed.translation = 0.2f;
  my_cartesian_speed.orientation = 50.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = collector_fetch_pose[0];
  my_constrained_pose.target_pose.y = collector_fetch_pose[1];
  my_constrained_pose.target_pose.z = collector_fetch_pose[2] + 0.1;
  my_constrained_pose.target_pose.theta_x = collector_fetch_pose[3];
  my_constrained_pose.target_pose.theta_y = collector_fetch_pose[4];
  my_constrained_pose.target_pose.theta_z = collector_fetch_pose[5];

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

  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  wait_for_action_end_or_abort();
  return true;
}


bool go_zhixue(ros::NodeHandle n, const std::string &robot_name)
{
  kortex_driver::ConstrainedPose my_constrained_pose;
  kortex_driver::CartesianSpeed my_cartesian_speed;

  std::vector<double> pose;
  pose = getzhixuepose(n);
  for(auto i : pose)
  {
    std::cout << i << std::endl;
  }

  my_cartesian_speed.translation = 0.08f;
  my_cartesian_speed.orientation = 70.0f;

  my_constrained_pose.constraint.oneof_type.speed.push_back(my_cartesian_speed);

  my_constrained_pose.target_pose.x = pose[0];;
  my_constrained_pose.target_pose.y = pose[1];
  my_constrained_pose.target_pose.z = pose[2] - 0.015;
  my_constrained_pose.target_pose.theta_x = pose[3];
  my_constrained_pose.target_pose.theta_y = pose[4];
  my_constrained_pose.target_pose.theta_z = pose[5];

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

bool set_payload(ros::NodeHandle n, const std::string &robot_name)
{

  kortex_driver::SetPayloadInformation req;
  kortex_driver::ControlConfig_Position position;
  position.x = 0.;
  position.y = 0.;
  position.z = 0.035;
  
  req.request.input.payload_mass = 1.45;
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

bool put_sensor(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = put_sensor_pose[0];
  cartesianWaypoint.pose.y = put_sensor_pose[1];
  cartesianWaypoint.pose.z = put_sensor_pose[2] + 0.2;
  cartesianWaypoint.pose.theta_x = angles::from_degrees(put_sensor_pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(put_sensor_pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(put_sensor_pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 30.;
  cartesianWaypoint.maximum_linear_velocity = 0.06;

  kortex_driver::CartesianWaypoint cartesianWaypoint1;

  cartesianWaypoint1.pose.x = put_sensor_pose[0];
  cartesianWaypoint1.pose.y = put_sensor_pose[1];
  cartesianWaypoint1.pose.z = put_sensor_pose[2];
  cartesianWaypoint1.pose.theta_x = angles::from_degrees(put_sensor_pose[3]);
  cartesianWaypoint1.pose.theta_y = angles::from_degrees(put_sensor_pose[4]);
  cartesianWaypoint1.pose.theta_z = angles::from_degrees(put_sensor_pose[5]);
  cartesianWaypoint1.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint1.blending_radius = 0;
  cartesianWaypoint1.maximum_angular_velocity = 40.;
  cartesianWaypoint1.maximum_linear_velocity = 0.1;

  goal.trajectory.push_back(cartesianWaypoint);
  goal.trajectory.push_back(cartesianWaypoint1);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}

bool put_sensor_up_and_go_up(ros::NodeHandle n, const std::string &robot_name)
{
  actionlib::SimpleActionClient<kortex_driver::FollowCartesianTrajectoryAction> ac(robot_name + "/cartesian_trajectory_controller/follow_cartesian_trajectory", true);
  ros::Duration server_timeout(5, 0);

  // wait for the action server to start
  ac.waitForServer();

  kortex_driver::FollowCartesianTrajectoryGoal goal;

  goal.use_optimal_blending = false;

  kortex_driver::CartesianWaypoint cartesianWaypoint;

  cartesianWaypoint.pose.x = put_sensor_pose[0];
  cartesianWaypoint.pose.y = put_sensor_pose[1];
  cartesianWaypoint.pose.z = put_sensor_pose[2] + 0.2;
  cartesianWaypoint.pose.theta_x = angles::from_degrees(put_sensor_pose[3]);
  cartesianWaypoint.pose.theta_y = angles::from_degrees(put_sensor_pose[4]);
  cartesianWaypoint.pose.theta_z = angles::from_degrees(put_sensor_pose[5]);
  cartesianWaypoint.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint.blending_radius = 0;
  cartesianWaypoint.maximum_angular_velocity = 30.;
  cartesianWaypoint.maximum_linear_velocity = 0.06;

  kortex_driver::CartesianWaypoint cartesianWaypoint1;

  cartesianWaypoint1.pose.x = 0.378;
  cartesianWaypoint1.pose.y = 0.005;
  cartesianWaypoint1.pose.z = 0.45;
  cartesianWaypoint1.pose.theta_x = angles::from_degrees(180.);
  cartesianWaypoint1.pose.theta_y = angles::from_degrees(0.);
  cartesianWaypoint1.pose.theta_z = angles::from_degrees(90.);
  cartesianWaypoint1.reference_frame = kortex_driver::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_BASE;
  cartesianWaypoint1.blending_radius = 0;
  cartesianWaypoint1.maximum_angular_velocity = 45.;
  cartesianWaypoint1.maximum_linear_velocity = 0.1;

  goal.trajectory.push_back(cartesianWaypoint);
  goal.trajectory.push_back(cartesianWaypoint1);
 
  ac.sendGoal(goal);

  //wait for the action to return
  bool completed = ac.waitForResult(ros::Duration(50.0));

  if (completed)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
  {
    ROS_ERROR("Action did not finish before the time out.");
  }
  //exit
  return completed;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "full_arm_cartesian_action_cpp");

  bool success = true;
  //*******************************************************************************
  // ROS Parameters
  ros::NodeHandle n;
  std::string robot_name = "my_gen3";

  ros::Subscriber sub = n.subscribe("/" + robot_name  + "/action_topic", 1000, notification_callback);

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
    success = false;
  }

  //init grip
  //=================================================================
  grip_init(n);
  grip_open();
    
  // success &= get_payload(n, robot_name);

  success &= set_payload(n, robot_name);
  //1. fetch lancet
  //=================================================================
  success &= go_lancet_up(n, robot_name);
  success &= go_lancet(n, robot_name);
  grip_close();

  //2. go infector pose
  //=================================================================
  success &= go_lancet_up_and_go_up(n, robot_name);
  // // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  success &= go_disinfector_up(n, robot_name);
  system("usbrelay 6QMBS_3=1");
  sleep(0.5);
  system("usbrelay 6QMBS_3=0");


  //3. finger tip blood
  //=================================================================
  success &= go_fingertip_up(n, robot_name);
  success &= force(n, robot_name);

  //4. push fingertip
  //=================================================================
  success &= go_fingertip_up_constrained_pose(n, robot_name);
  // sleep(2);
  success &= pushbeam_up(n, robot_name);
  sleep(8);

  //5. put lancet and fetch sensor
  //=================================================================
  success &= after_push_up(n, robot_name);
  success &= put_lancet(n, robot_name);
  grip_open();
  sleep(1);   
  success &= go_sensor_up(n, robot_name);
  success &= force_sensor(n, robot_name);
  // sleep(2);
  success &= go_sensor_up_later(n, robot_name);
  success &= collector_up(n, robot_name);

  //6. collector blood 
  //=================================================================
  success &= force2(n, robot_name);
  
  //7. zhixue
  //=================================================================
  success &= collector_up(n, robot_name);
  success &= go_zhixue(n, robot_name);
  system("usbrelay 6QMBS_4=1");
  sleep(0.5);
  system("usbrelay 6QMBS_4=0");

  //8. put sensor
  //=================================================================
  success &= put_sensor(n, robot_name);
  success &= put_sensor_up_and_go_up(n, robot_name);
  

  success &= all_notifs_succeeded;

  // Report success for testing purposes
  ros::param::set("/kortex_examples_test_results/cartesian_poses_with_notifications_cpp", success);
  
  return success ? 0 : 1;
}