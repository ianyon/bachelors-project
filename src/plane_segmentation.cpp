#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>

using namespace pcl;
using namespace std;

PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
const PointCloud<PointXYZ>::Ptr cloud_p(new PointCloud<PointXYZ>);
const PointCloud<PointXYZ>::Ptr croppedCloud(new PointCloud<PointXYZ>);
bool update;

void visualize()
{
    visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer ("PCL Viewer"));
    viewer->setBackgroundColor (0.2, 0.2, 0.2);
    // red (x), green (y), and blue (z).
    viewer->addCoordinateSystem (0.3);
    viewer->initCameraParameters();


    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        // Check if cloud was updated (If not present program fails due to try to visualize zero size normals)
        if(update)
        {
            visualization::PointCloudColorHandlerCustom<PointXYZ> single_color(cloud_p, 0, 255, 0);
            if(!viewer->updatePointCloud(cloud_p, single_color, "plane"))
                viewer->addPointCloud<PointXYZ>(cloud_p,  single_color, "plane");

            if(!viewer->updatePointCloud(croppedCloud, "Cloud"))
                viewer->addPointCloud<PointXYZ>(croppedCloud, "Cloud");
            update = false;
        }
    }
}

double durationInMillis(clock_t &begin)
{
    return (double(clock() - begin) / CLOCKS_PER_SEC)*1000;
}

void sensorCallback (const PCLPointCloud2::ConstPtr& sensorInput)
{
    ROS_INFO_ONCE("Callback Called");
    clock_t beginCallback = clock();

    fromPCLPointCloud2 (*sensorInput,*cloud);

    // Kinect is 640/480
    int xRealSize = 64*2,
            yRealSize = 48*2;
    int xInit = fmax(cloud->width/2 - xRealSize/2, 0),
            yInit = fmax(cloud->height/2 - yRealSize/2, 0);
    int xEnd = fmin(xInit + xRealSize, cloud->width),
            yEnd = fmin(yInit + yRealSize, cloud->height);

    // Reserve space in the points vector to store the cloud
    croppedCloud->reserve(xRealSize*yRealSize);
    // Make que dimensions of the cloud be according the size of the vector
    croppedCloud->width = xRealSize;
    croppedCloud->height = yRealSize;
    croppedCloud->sensor_origin_ = cloud->sensor_origin_;
    croppedCloud->sensor_orientation_ = cloud->sensor_orientation_;

    for (size_t i=xInit; i < xEnd; ++i)
    {
        for (size_t j=yInit ; j < yEnd; ++j)
        {
            // We can't use croppedCloud->push_back because it breaks the organization
            croppedCloud->points.push_back(cloud->at(i, j));
        }
    }

    ROS_INFO("Cropped from %lu to %lu", cloud->points.size(), croppedCloud->points.size());

    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr inliers (new PointIndices);
    // Create the segmentation object
    SACSegmentation<PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setDistanceThreshold (0.005);

    // Create the filtering object
    ExtractIndices<PointXYZ> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (croppedCloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        ROS_INFO("CALLBACK DURATION %gms",durationInMillis(beginCallback));
        return;
    }

    // Extract the inliers
    extract.setInputCloud (croppedCloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    ROS_DEBUG_STREAM("PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points.");


    update = true;
    ROS_INFO("CALLBACK DURATION %gms",durationInMillis(beginCallback));
}

main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "bachelors_final_project");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, &sensorCallback);

    //Start visualizer thread
    boost::thread workerThread(&visualize);
    workerThread.detach();

    ROS_INFO("Escuchando");

    // Spin
    ros::spin();        // Handle ROS events

    return (0);
}

