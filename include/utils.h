#ifndef UTILS
#define UTILS

// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace pcl;

namespace utils
{

//! Publish the message.
void publish(const ros::Publisher &pub, const PointCloud<PointXYZ>::Ptr &cloud);

//! Compute the time since "begin"
double durationInMillis(clock_t &begin);
}

#endif // UTILS

