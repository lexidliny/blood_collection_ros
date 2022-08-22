#include <headDetec/headDetect.h>
#include <rs2pcd/rs2pcd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include <facemesh/IrisLandmark.hpp>

using namespace std;
using namespace cv;
using namespace cv::face;




void MM::head::headDetect::setImage(cv::Mat& color_image, cv::Mat& depth_image) 
{
    _raw_color_image = color_image;
    _raw_depth_image = depth_image;
}

void MM::head::headDetect::getheadPoint(my::IrisLandmark& irisLandmarker)
{
    
    // CascadeClassifier faceDetector("/home/yxw/Project/blood_collection/face_landmark/lbpcascade_frontalface.xml");

    // Ptr<Facemark> facemark = FacemarkLBF::create();

    // facemark->loadModel("/home/yxw/Project/blood_collection/face_landmark/lbfmodel.yaml");

    cv::Mat  gray;
    _detected_image = _raw_color_image.clone();

    
    vector<Rect> faces;
    
    irisLandmarker.loadImageToInput(_detected_image);
    irisLandmarker.runInference();  
    int i = 0;
    cv::Point2d face_model_points;

    for (auto landmark: irisLandmarker.getAllFaceLandmarks()) {
        cv::circle(_detected_image, landmark, 2, cv::Scalar(0, 255, 0), -1);
        if (i == 151){
            _head_center_point = landmark;
            cv::circle(_detected_image, landmark, 6, cv::Scalar(0, 0, 255), -1);
            cout << landmark << endl;
        }
        if (i == 9 ){
            _head_center_point_former = landmark;
            cv::circle(_detected_image, landmark, 6, cv::Scalar(0, 0, 255), -1);
            cout << landmark << endl;
        }
        i++;
    }

    
    std::cout << "get head center Point: " <<  _head_center_point << std::endl;
}




void MM::head::headDetect::getheadPose(rs2::depth_frame& depth)
{  
    // calculate the search point from watch_center_point

    // distance at z axis from camera to watch_center_point
    auto dist = depth.get_distance(_head_center_point.x, _head_center_point.y);

    float pointxyz[3];
    float pix[2] = {_head_center_point.x, _head_center_point.y};
    // Get the coordinate of watch_center_point in xyz frame
    rs2_deproject_pixel_to_point(pointxyz, &_intrin, pix, dist);
    
    // Get the search point in point cloud
    _search_point.x = pointxyz[0];
    _search_point.y = pointxyz[1];
    _search_point.z = pointxyz[2];

    auto dist2 = depth.get_distance(_head_center_point_former.x, _head_center_point_former.y);

    float pointxyz2[3];
    float pix2[2] = {_head_center_point_former.x, _head_center_point_former.y};
    // Get the coordinate of watch_center_point in xyz frame
    rs2_deproject_pixel_to_point(pointxyz2, &_intrin, pix2, dist2);
    
    // Get the search point in point cloud
    _search_point_former.x = pointxyz2[0];
    _search_point_former.y = pointxyz2[1];
    _search_point_former.z = pointxyz2[2];

    // Normal estimation

    // normal class
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    // buid kdtree for searching points around watch_center_point
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    
    // Set point cloud for kdtree
    tree->setInputCloud (_pointcloud);
    n.setSearchMethod(tree);
    
    int K = 100;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::cout << "K nearest neighbor search at (" << _search_point.x 
        << " " << _search_point.y 
        << " " << _search_point.z
        << ") with K=" << K << std::endl;
    if ( tree->nearestKSearch (_search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            std::cout << "    "  << _pointcloud->points[ pointIdxNKNSearch[i] ].x 
            << " " << _pointcloud->points[ pointIdxNKNSearch[i] ].y 
            << " " << _pointcloud->points[ pointIdxNKNSearch[i] ].z 
            << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    }
    float cur;
    n.computePointNormal(*_pointcloud, pointIdxNKNSearch, _normal , cur);
    std::cout << "normal's values: " << '\n' << _normal << std::endl;
}



void MM::head::headDetect::showDetectedImage() const
{
    imshow("Facial Landmark Detection", _detected_image);
    
    // cv::waitKey(0);
}


void MM::head::headDetect::showDetectedPointCloud()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("frame show"));

    // Set background of viewer to black
    viewer->setBackgroundColor (0, 0, 0); 
    // Add generated point cloud and identify with string "Cloud"
    viewer->addPointCloud<pcl::PointXYZRGB> (_pointcloud, "frame show");
    // Default size for rendered points
    viewer->setPointCloudRenderingProperties (
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "frame show");
        
    pcl::PointXYZ p1;
    pcl::PointXYZ p2;

    p1.x = _search_point.x;
    p1.y = _search_point.y;
    p1.z = _search_point.z;
    p2.x = p1.x - 0.3 * _normal[0];
    p2.y = p1.y - 0.3 * _normal[1];
    p2.z = p1.z - 0.3 * _normal[2];

    viewer->addLine(p1, p2, 255, 0 ,0, "nz");
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "nz");

    pcl::PointXYZ projectedPoint = getProjectedPoint(_search_point_former, _normal);
    Eigen::Vector3f nx_input = {
        projectedPoint.x - p1.x,
        projectedPoint.y - p1.y,
        projectedPoint.z - p1.z,
    };
    getAffine(nx_input, p1);
    viewer->addCoordinateSystem(0.2, _affine);

    
    viewer->addCoordinateSystem();
    // Viewer Properties
    viewer->initCameraParameters();  // Camera Parameters for ease of viewing

    cout << "control + c to exit the program. " << endl;
     viewer->spinOnce(10000);
    
    viewer->close();
}

double MM::head::headDetect::getDistance(cv::Point2f point1, cv::Point2f point2)
{
    double distance = sqrtf(powf((point1.x - point2.x), 2) 
                        + powf((point1.y - point2.y), 2));
    return distance;
}

void MM::head::headDetect::setPointCloud(
    rs2::video_frame& RGB, rs2::depth_frame& depth)
{
    // get point cloud from realsense
    _pointcloud = ::rs2pcd(RGB, depth);

    // get intrinsic of depth frame
    auto color_stream = RGB.get_profile().as<rs2::video_stream_profile>();
    _intrin = color_stream.get_intrinsics();
}


pcl::PointXYZ MM::head::headDetect::getProjectedPoint(pcl::PointXYZ& xAxisPointOutPlane, Eigen::Vector4f& planeCoe)
{
    pcl::PointXYZ xAisPointInPlane;
    double A = planeCoe[0];
    double B = planeCoe[1];
    double C = planeCoe[2];
    double D = planeCoe[3];
    double abd_2 = planeCoe[0] * planeCoe[0] + planeCoe[1] * planeCoe[1] + planeCoe[2] * planeCoe[2];

    xAisPointInPlane.x = 
    ((B*B+C*C)*xAxisPointOutPlane.x - A*(B*xAxisPointOutPlane.y + C*xAxisPointOutPlane.z + D))/abd_2;
    xAisPointInPlane.y = 
    ((A*A +C*C)*xAxisPointOutPlane.y -B*(A*xAxisPointOutPlane.x+C*xAxisPointOutPlane.z +D))/abd_2;
    xAisPointInPlane.z = 
    ((A*A+B*B)*xAxisPointOutPlane.z - C*(A*xAxisPointOutPlane.x + B*xAxisPointOutPlane.y +D)) /abd_2;

    return xAisPointInPlane;
}

void MM::head::headDetect::getAffine(Eigen::Vector3f& nx_input, pcl::PointXYZ& origin)
{
    Eigen::Vector3f nx, ny, nz, t;
    nx = nx_input;
    nz = {_normal[0], _normal[1], _normal[2]};
    ny = nz.cross(nx);
    nx = ny.cross(nz);
    double nx_mode = std::sqrt(nx[0]*nx[0] + nx[1]*nx[1] + nx[2]*nx[2]);
    double ny_mode = std::sqrt(ny[0]*ny[0] + ny[1]*ny[1] + ny[2]*ny[2]);
    double nz_mode = std::sqrt(nz[0]*nz[0] + nz[1]*nz[1] + nz[2]*nz[2]);



    Eigen::Vector4d nx_homo, ny_homo, nz_homo, t_homo;
    nx_homo = {nx[0]/nx_mode, nx[1]/nx_mode, nx[2]/nx_mode, 0};
    ny_homo = {ny[0]/ny_mode, ny[1]/ny_mode, ny[2]/ny_mode, 0};
    nz_homo = {nz[0]/nz_mode, nz[1]/nz_mode, nz[2]/nz_mode, 0};

    origin.x -= 0.05 * nz_homo[0];
    origin.y -= 0.05 * nz_homo[1];
    origin.z -= 0.05 * nz_homo[2];

    t_homo = {origin.x, origin.y, origin.z, 1};
    
    Eigen::Matrix4d tran;

    tran(0, 0) = nx_homo[0];  
    tran(0, 1) = ny_homo[0];
    tran(0, 2) = nz_homo[0];
    tran(0, 3) = t_homo[0];

    tran(1, 0) = nx_homo[1];
    tran(1, 1) = ny_homo[1];
    tran(1, 2) = nz_homo[1];
    tran(1, 3) = t_homo[1];
    
    tran(2, 0) = nx_homo[2];
    tran(2, 1) = ny_homo[2];
    tran(2, 2) = nz_homo[2];
    tran(2, 3) = t_homo[2];

    tran(3, 0) = nx_homo[3];
    tran(3, 1) = ny_homo[3];
    tran(3, 2) = nz_homo[3];
    tran(3, 3) = t_homo[3];

    Eigen::Matrix4f tranf= tran.cast<float>();


    _affine = tranf;
    std::cout << "The watch wearing point affine matrix: " << '\n' << _affine.matrix() << std::endl;
}

void MM::head::headDetect::matrix2quaternion(Eigen::Matrix4d& tran, double* quat)
{
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = tran.block(0,0,3,3);

    Eigen::Quaterniond quaternion;
    quaternion=rotation_matrix;
    
    quat[0] = tran(0,3);
    quat[1] = tran(0,3);
    quat[2] = tran(0,3);
    quat[3] = quaternion.x();
    quat[4] = quaternion.y();
    quat[5] = quaternion.z();
    quat[6] = quaternion.w();
}

void MM::head::headDetect::quaternion2matrix(Eigen::Matrix4d& tran, double* quat)
{

    Eigen::Matrix3d rotation_matrix;

    Eigen::Quaterniond quaternion;
    quaternion.x() = quat[3];
    quaternion.x() = quat[4];
    quaternion.x() = quat[5];
    quaternion.x() = quat[6];

    rotation_matrix = quaternion.matrix();
    
    tran.block(0,0,3,3) = rotation_matrix;

    tran(0,3) = quat[0];
    tran(1,3) = quat[1];
    tran(2,3) = quat[2];
    tran(3,0) = 0;
    tran(3,1) = 0;
    tran(3,2) = 0;
    tran(3,3) = 1;
}

MM::head::headDetect::~headDetect()
{

}
