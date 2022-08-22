#ifndef HEADDETECT_H
#define HEADDETECT_H

#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <facemesh/IrisLandmark.hpp>



namespace MM{

namespace head{

    class headDetect
    {
        public:
            cv::Mat _raw_color_image;
            cv::Mat _raw_depth_image;
            cv::Mat _aligned_depth_image;
            cv::Mat _detected_image;
        
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointcloud;

            pcl::PointXYZRGB _search_point;    // watch center point in pcl
            pcl::PointXYZ _search_point_former;
            cv::Point2f _head_center_point;    //watch center point in image
            cv::Point2f _head_center_point_former;
            Eigen::Vector4f _normal;
            Eigen::Affine3f _affine;



            
        public:
            headDetect():_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>),config(0){}

            headDetect(cv::Mat& color_image):_raw_color_image(color_image),
                        _pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>){}

            ~headDetect();

            // Set Image
            void setImage(cv::Mat& color_image, cv::Mat& depth_iamge);
            // Set Point cloud
            void setPointCloud(rs2::video_frame&, rs2::depth_frame&);
            // Get wrist point
            void getheadPoint(my::IrisLandmark& irisLandmarker);
            //Get wrist pose
            void getheadPose(rs2::depth_frame&);
 
            
            // Show kinds of image
            void showDetectedImage() const;
            
            void showDetectedPointCloud();

        public:

            rs2_intrinsics _intrin;
            int config;

            double getDistance(cv::Point2f point1, cv::Point2f point2);
            pcl::PointXYZ getProjectedPoint(pcl::PointXYZ&, Eigen::Vector4f&);
            
            void getAffine(Eigen::Vector3f&, pcl::PointXYZ&);

            static void matrix2quaternion(Eigen::Matrix4d&, double*);
            static void quaternion2matrix(Eigen::Matrix4d&, double*);
    };


};
};

#endif