#include <ros/ros.h>
#include <facemesh/IrisLandmark.hpp>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <headDetec/headDetect.h>
#include <librealsense2/rs.hpp>
#include <forehead_detect/GetForeheadPose.h>

bool getForeheadPoseCB(forehead_detect::GetForeheadPose::Request &request, forehead_detect::GetForeheadPose::Response& res)
{   
    my::IrisLandmark irisLandmarker("/home/kortex/yxw/blood_collection_ros/src/bood_collection/models");
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 15);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
    pipe.start(cfg);

    while (1)
    {
        // align depth frame to depth frame

        rs2::frameset frames = pipe.wait_for_frames();
        rs2::align align_to_color(RS2_STREAM_COLOR);
        rs2::frameset alignedframe = align_to_color.process(frames);

        auto depth = alignedframe.get_depth_frame();
        auto color = alignedframe.get_color_frame();
        
        const int w_c = color.as<rs2::video_frame>().get_width();
        const int h_c = color.as<rs2::video_frame>().get_height();

        cv::Mat image_color(cv::Size(w_c, h_c), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat image_depth(cv::Size(w_c, h_c), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

        MM::head::headDetect dec1;

        dec1.setImage(image_color, image_depth);
        dec1.getheadPoint(irisLandmarker);
        dec1.showDetectedImage();

        if (cv::waitKey(1) == 112) // low case 'p' key
        {
            dec1.setPointCloud(color, depth);
            dec1.getheadPose(depth);
            dec1.showDetectedPointCloud();



            tf::TransformListener listener;
            std::cout<<"1. 阻塞直到root 和 realsens  frame相通"<<std::endl;
            listener.waitForTransform("base_link","realsensed455",ros::Time(0),ros::Duration(4.0));
            tf::StampedTransform realsense2root;
            try{
             //2. 监听对应的tf,返回平移和旋转
            std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
            listener.lookupTransform("/base_link", "/realsensed455",
                ros::Time(0), realsense2root);
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            tf::StampedTransform ee2realsense;
            listener.waitForTransform("realsensed455","new_end_effector_link",ros::Time(0),ros::Duration(4.0));
            try{
                    //2. 监听对应的tf,返回平移和旋转
                    std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
                    listener.lookupTransform("/realsensed455", "/new_end_effector_link",
                    ros::Time(0), ee2realsense);
                }
            catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
            }
            

            tf::StampedTransform ee2caixuezhen;
            listener.waitForTransform("caixuezhen","new_end_effector_link",ros::Time(0),ros::Duration(4.0));
            try{
                    //2. 监听对应的tf,返回平移和旋转
                    std::cout<<"2. 监听对应的tf,返回平移和旋转"<<std::endl;
                    listener.lookupTransform("/caixuezhen", "/new_end_effector_link",
                    ros::Time(0), ee2caixuezhen);
                }
            catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
            }
            

     
            tf::Transform realsense2root_tf;
            tf::Transform ee2realsense_tf;
            tf::Transform ee2caixuezhen_tf;
            tf::Transform target2realsense_tf;
     
            tf::Transform target2root_tf;
            tf::Transform ee2root_tf;
            
            tf::Vector3 t(dec1._affine.translation().x(), 
            dec1._affine.translation().y(), 
            dec1._affine.translation().z());


            tf::Matrix3x3 affine(
                dec1._affine.rotation().matrix()(0,0),
                dec1._affine.rotation().matrix()(0,1),
                dec1._affine.rotation().matrix()(0,2),
                dec1._affine.rotation().matrix()(1,0),
                dec1._affine.rotation().matrix()(1,1),
                dec1._affine.rotation().matrix()(1,2),
                dec1._affine.rotation().matrix()(2,0),
                dec1._affine.rotation().matrix()(2,1),
                dec1._affine.rotation().matrix()(2,2)
            );
            
            
            tf::Quaternion q2; 
            affine.getRotation(q2);
            target2realsense_tf.setOrigin(t);
            target2realsense_tf.setRotation(q2);

            
            realsense2root_tf.setOrigin(realsense2root.getOrigin());
            realsense2root_tf.setRotation(realsense2root.getRotation());
            
            ee2realsense_tf.setOrigin(ee2realsense.getOrigin());
            ee2realsense_tf.setRotation(ee2realsense.getRotation());

            ee2caixuezhen_tf.setOrigin(ee2caixuezhen.getOrigin());
            ee2caixuezhen_tf.setRotation(ee2caixuezhen.getRotation());

            target2root_tf.mult(realsense2root_tf,target2realsense_tf);
            ee2root_tf.mult(target2root_tf, ee2caixuezhen_tf);


            // tf::Transform tar2ee_tf;
            // tar2ee_tf.mult( ee2realsense.inverse() ,target2realsense_tf);
            
            res.ForeheadPose.position.x = ee2root_tf.getOrigin().x();
            res.ForeheadPose.position.y = ee2root_tf.getOrigin().y();
            res.ForeheadPose.position.z = ee2root_tf.getOrigin().z();
            res.ForeheadPose.orientation.x = ee2root_tf.getRotation().getX();
            res.ForeheadPose.orientation.y = ee2root_tf.getRotation().getY();
            res.ForeheadPose.orientation.z = ee2root_tf.getRotation().getZ();
            res.ForeheadPose.orientation.w = ee2root_tf.getRotation().getW();

            // res.ForeheadPose.position.x = tar2ee_tf.getOrigin().x();
            // res.ForeheadPose.position.y = tar2ee_tf.getOrigin().y();
            // res.ForeheadPose.position.z = tar2ee_tf.getOrigin().z();
            // res.ForeheadPose.orientation.x = tar2ee_tf.getRotation().getX();
            // res.ForeheadPose.orientation.y = tar2ee_tf.getRotation().getY();
            // res.ForeheadPose.orientation.z = tar2ee_tf.getRotation().getZ();
            // res.ForeheadPose.orientation.w = tar2ee_tf.getRotation().getW();
            
            break;
        }
        // cv::destroyAllWindows();
        std::cout << "======================================" << std::endl;
        
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"get_forehead_pose_server");
  ros::NodeHandle n;

  
  ros::ServiceServer service = n.advertiseService("get_forehead_pose_server", &getForeheadPoseCB);
  

  ROS_INFO("get_forehead_pose_server start.");


  ros::spin();

  return 0;
}
