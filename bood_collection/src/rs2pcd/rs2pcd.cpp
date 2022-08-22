#include <rs2pcd.h>

using namespace std;





std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    cloud_pointer cloud(new point_cloud);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

    }
    
   return cloud; // PCL RGB Point Cloud generated
}

void Load_PCDFile(void)
{
    std::string openFileName;

    // Generate object to store cloud in .pcd file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudView (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    openFileName = "Captured_Frame.pcd";
    pcl::io::loadPCDFile (openFileName, *cloudView); // Load .pcd File
    
    //==========================
    // Pointcloud Visualization
    //==========================
    // Create viewer object titled "Captured Frame"
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Captured Frame"));

    // Set background of viewer to black
    viewer->setBackgroundColor (0, 0, 0); 
    // Add generated point cloud and identify with string "Cloud"
    viewer->addPointCloud<pcl::PointXYZRGB> (cloudView, "Cloud");
    // Default size for rendered points
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
    // Viewer Properties
    viewer->initCameraParameters();  // Camera Parameters for ease of viewing

    cout << endl;
    cout << "Press [Q] in viewer to continue. " << endl;
    
    viewer->spin(); // Allow user to rotate point cloud and view it

    // Note: No method to close PC visualizer, pressing Q to continue software flow only solution.
   
    
}

cloud_pointer rs2pcd(rs2::video_frame& RGB, rs2::depth_frame& depth)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;

    
    // Map Color texture to each point
    pc.map_to(RGB);

    // Generate Point Cloud
    auto points = pc.calculate(depth);
    // Convert generated Point Cloud to PCL Formatting
    cloud = PCL_Conversion(points, RGB);

    //========================================
    // Filter PointCloud (PassThrough Method)
    //========================================
    pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter;    // Create the filtering object
    Cloud_Filter.setInputCloud (cloud);                 // Input generated cloud to filter
    Cloud_Filter.setFilterFieldName ("z");              // Set field name to Z-coordinate
    Cloud_Filter.setFilterLimits (0.0, 1.0);            // Set accepted interval values
    Cloud_Filter.filter (*newcloud);                    // Filtered Cloud Outputted    
    
    cloudFile = "Captured_Frame.pcd";

    //==============================
    // Write PC to .pcd File Format
    //==============================
    // Take Cloud Data and write to .PCD File Format
    cout << "Generating PCD Point Cloud File... " << endl;
    pcl::io::savePCDFileASCII(cloudFile, *cloud); // Input cloud to be saved to .pcd
    cout << cloudFile << " successfully generated. " << endl;
    std::cout << "cloud's size" << (*cloud).size() << std::endl;
    return newcloud;

}