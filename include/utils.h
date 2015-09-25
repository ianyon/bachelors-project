#ifndef UTILS
#define UTILS

// ROS includes
#include <ros/ros.h>

// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace utils
{

//! Publish the message.
void publish(const ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

//! Compute the time since "begin"
double durationInMillis(clock_t &begin);
}

#endif // UTILS

