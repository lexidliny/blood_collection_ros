#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <kortex_driver/BaseCyclic_Feedback.h>

using namespace std;

//退出用：ctrl+z

int main(int argc, char** argv){
//初始化
  ros::init(argc, argv, "new_end_effector_tf_broadcaster");
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

  ros::NodeHandle node;

  ros::Rate rate(400);
  while(ros::ok())
  {
    auto feedback = ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/" + robot_name + "/base_feedback");
  // Initialize the ServiceClient

  // Initialize input
    double current_x = feedback->base.commanded_tool_pose_x ;
    double current_y = feedback->base.commanded_tool_pose_y ;
    double current_z = feedback->base.commanded_tool_pose_z ;
    double current_theta_x = feedback->base.commanded_tool_pose_theta_x * M_PI / 180;
    double current_theta_y = feedback->base.commanded_tool_pose_theta_y * M_PI / 180;
    double current_theta_z = feedback->base.commanded_tool_pose_theta_z * M_PI / 180;

    static tf::TransformBroadcaster br;
    tf::Transform  ee2base_tf;

    ee2base_tf.setOrigin(tf::Vector3(current_x, current_y, current_z));
    tf::Quaternion q;
    // q.setRPY(current_theta_x,current_theta_y,current_theta_z);
    q.setRPY( current_theta_x ,current_theta_y, current_theta_z);
  
    ee2base_tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(ee2base_tf,ros::Time::now(),"base_link","new_end_effector_link"));
    std::cout<< "x=" << current_x <<", y=" << current_y <<", z=" << current_z <<", theta_x=" << current_theta_x 
    <<", theta_y=" << current_theta_y <<", theta_z=" << current_theta_z << endl;
    rate.sleep();
  }
  return 0;
}