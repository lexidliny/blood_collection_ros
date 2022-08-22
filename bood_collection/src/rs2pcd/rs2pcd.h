#ifndef RS2PCD_H
#define RS2PCD_H


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <librealsense2/rs.hpp> 
#include <string>
#include <iostream>
#include <algorithm>

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;

using namespace std;

// Global Variables
static string cloudFile; // .pcd file name



std::tuple<int, int, int> RGB_Texture(
    rs2::video_frame texture, rs2::texture_coordinate Texture_XY);

cloud_pointer PCL_Conversion(
    const rs2::points& points, const rs2::video_frame& color);

void Load_PCDFile(void);

cloud_pointer rs2pcd(rs2::video_frame&, rs2::depth_frame&);

#endif
