//
// Created by lars on 08.08.19.
//
#include <ros/ros.h>
#include <ros/time.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <marker_helper/markerHelper.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <detection_msgs/DetectedObjects.h>

#include "../include/cloud.h"
#include "../include/medianFilter.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = nullptr;
tf2_ros::Buffer tfBuffer;
std::string TARGET_FRAME = "pickUpLeft";
ros::Time sceneTime;

tf2::Vector3 startBox = tf2::Vector3(0.0,0.0,0.005);
tf2::Vector3 endBox = tf2::Vector3(0.18,0.4,0.15);

const size_t SAMPLE_SIZE = 10;

double thresholdA = 0.15;
double thresholdB = 0.7;
double radius = 0.0075;
int regionColorThreshold = 5;
int pointColorThreshold = 6;
int k = 20;
int minClusterSize = 20;
int maxClusterSize = 500;
double regionDistanceThreshold = 0.02;
bool enableColorSegmentation = true;

std::unique_ptr<MarkerHelper> markerManager;
std::shared_ptr<ros::NodeHandle> nh;

void pointcloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(msg->width == 0) return;

    sensor_msgs::PointCloud2 other;
    geometry_msgs::TransformStamped tS;
    try{
        tS = tfBuffer.lookupTransform(TARGET_FRAME,msg->header.frame_id,ros::Time(0));
        tf2::doTransform(*msg,other,tS);
        sceneTime = msg->header.stamp;
        pcl::fromROSMsg(other,*cloud1);
    }catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
}

int main (int argc, char** argv) {
    // Load input file into a PointCloud<T> with an appropriate type
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PCLPointCloud2 cloud_blob;
    //pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
    //pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    //* the data should be available in cloud

    std::string applicationName = "rotationDetection";
    if (argc > 1) applicationName = argv[1];
    else std::cout << "Usage: rosrun detection graspPointDetection [NodeName] [TargetFrame]\n";
    if (argc > 2) TARGET_FRAME = argv[2];

    ros::init(argc, argv, "rotationDetection");
    nh = std::make_shared<ros::NodeHandle>();

    nh->param("/" + applicationName + "/thresholdA", thresholdA);
    nh->param("/" + applicationName + "/thresholdB", thresholdB);
    nh->param("/" + applicationName + "/radius", radius);
    nh->param("/" + applicationName + "/colorSegmentation", enableColorSegmentation);
    nh->param("/" + applicationName + "/startX", startBox.x());
    nh->param("/" + applicationName + "/startY", startBox.y());
    nh->param("/" + applicationName + "/startZ", startBox.z());
    nh->param("/" + applicationName + "/endX", endBox.x());
    nh->param("/" + applicationName + "/endY", endBox.y());
    nh->param("/" + applicationName + "/endZ", endBox.z());
    nh->param("/" + applicationName + "/pointColorThreshold", pointColorThreshold);
    nh->param("/" + applicationName + "/regionColorThreshold", regionColorThreshold);
    nh->param("/" + applicationName + "/minClusterSize", minClusterSize);
    nh->param("/" + applicationName + "/maxClusterSize", maxClusterSize);

    ros::Subscriber sub;

    cloud1 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    tf2_ros::TransformListener tfListener(tfBuffer);
    markerManager = std::make_unique<MarkerHelper>(nh, "/" + applicationName + "/marker", "objects");

    sub = nh->subscribe<sensor_msgs::PointCloud2>("/kinect2/hd/points", 1, pointcloudCallback);
    ros::Publisher objectsPub = nh->advertise<detection_msgs::DetectedObjects>("/" + applicationName + "/objects", 10);
    ros::Publisher debugCloudPub = nh->advertise<sensor_msgs::PointCloud2>("/" + applicationName + "/debugCloud", 2);

    ros::Rate r(4);
    std::cout << "Starting loop!\n";

    cloud::MedianFilter mf;

    for (size_t i = 0; i < SAMPLE_SIZE; i++) {
        while (cloud1 == nullptr || cloud1->size() == 0) {
            ros::spinOnce();
        }
        mf.push_back(cloud1);
        cloud1 = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        std::cout << "Set sample number: " << i+1 << std::endl;
    }

    mf.compute(cloud1);

    std::cout << "Pointcloud data retrieved" << std::endl;

    //Using a box cut to only keep points inside the pickup area.
    pcl::PointCloud<cloud::PointType>::Ptr cutScene = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
    cloud::cutPointCloudRect(cloud1,cutScene,startBox,endBox);

    //Using normal segmentation to only keep points which face upwoards and therefore can be grasped.
    pcl::PointCloud<cloud::PointType>::Ptr normalSeg = boost::make_shared<pcl::PointCloud<cloud::PointType>>();
    cloud::normalBasedSegmentation(cutScene,normalSeg,tf2::Vector3(0,0,1),thresholdA,thresholdB,radius,k);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*normalSeg, *cloud2);
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud2);

        // Normal estimation*
        pcl::NormalEstimation <pcl::PointXYZ, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZ>);
        tree->setInputCloud(cloud2);
        n.setInputCloud(cloud2);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.compute(*normals);
        //* normals should not contain the point normals + surface curvatures

        // Concatenate the XYZ and normal fields*
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud <pcl::PointNormal>);
        pcl::concatenateFields(*cloud2, *normals, *cloud_with_normals);
        //* cloud_with_normals = cloud + normals

        // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree <pcl::PointNormal>);
        tree2->setInputCloud(cloud_with_normals);

        // Initialize objects
        pcl::GreedyProjectionTriangulation <pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius(0.025);

        // Set typical values for the parameters
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
        gp3.setMinimumAngle(M_PI / 18); // 10 degrees
        gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
        gp3.setNormalConsistency(false);

        // Get result
        gp3.setInputCloud(cloud_with_normals);
        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);

        // Additional vertex information
        std::vector<int> parts = gp3.getPartIDs();
        std::vector<int> states = gp3.getPointStates();

    pcl::io::saveVTKFile ("mesh.vtk", triangles);

    ros::shutdown();
    // Finish
    return (0);
}