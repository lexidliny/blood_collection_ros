#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


using namespace std;

//退出用：ctrl+z

int main(int argc, char** argv){
//初始化
    ros::init(argc, argv, "caixuezhen_tf_broadcaster");
    ros::NodeHandle node;
    double current_x = 0.0695 ;
    double current_y = 0;
    double current_z = 0.0671;
    double current_theta_x = 0;
    double current_theta_y = 0;
    double current_theta_z = 0. / 180.00 * M_PI;

    static tf::TransformBroadcaster br;
    tf::Transform  target2base_tf;

    target2base_tf.setOrigin(tf::Vector3(current_x, current_y, current_z));
    tf::Quaternion q;
    // q.setRPY(current_theta_x,current_theta_y,current_theta_z);
    q.setRPY( current_theta_x ,current_theta_y, current_theta_z);

    target2base_tf.setRotation(q);
    ros::Rate rate(400);
    while(ros::ok())
    {
    
    // Initialize input
    
        br.sendTransform(tf::StampedTransform(target2base_tf,ros::Time::now(),"new_end_effector_link","target_link"));
        std::cout<< "x=" << current_x <<", y=" << current_y <<", z=" << current_z <<", theta_x=" << current_theta_x 
        <<", theta_y=" << current_theta_y <<", theta_z=" << current_theta_z << endl;
        rate.sleep();
    }
    return 0;
}