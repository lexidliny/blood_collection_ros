#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "kinova_tf_listener");
  ros::NodeHandle node;
  tf::TransformListener listener;
  //1. 阻塞直到frame相通
  std::cout<<"1. 阻塞直到frame相通"<<std::endl;
  listener.waitForTransform("base_link","new_end_effector_link",ros::Time(0),ros::Duration(4.0));
  ros::Rate rate(1);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
  //2. 监听对应的tf,返回平移和旋转
      std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
      listener.lookupTransform("/base_link", "/new_end_effector_link",
                                ros::Time(0), transform);
                                //ros::Time(0)表示最近的一帧坐标变换，不能写成ros::Time::now()
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    tf::Quaternion q(transform.getRotation().getX(), transform.getRotation().getY(),
                  transform.getRotation().getZ(),transform.getRotation().getW());
    double roll, pitch, yaw;//x y z
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    roll = roll / M_PI * 180;
    pitch = pitch / M_PI * 180;
    yaw = yaw / M_PI * 180;
    std::cout<<"输出的位置坐标：x="<<transform.getOrigin().x()<<",y="<<transform.getOrigin().y()<<",z="<<transform.getOrigin().z()<<std::endl;
    std::cout<<"输出的旋转四元数：w="<<transform.getRotation().getW()<<",x="<<transform.getRotation().getX()<<
    ",y="<<transform.getRotation().getY()<<",z="<<transform.getRotation().getZ()<<std::endl;
    std::cout<<"rotation：theta_x="<<roll<<",theta_y="<<pitch<<
    ",theta_z="<<yaw<<std::endl;


    rate.sleep();
  }
  return 0;
};