#ifndef UTILS
#define UTILS

// ROS includes
#include <ros/ros.h>

// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace bachelors_final_project
{
namespace segmentation
{

//! Publish the message.
void publish(const ros::Publisher &, const pcl::PointCloud<pcl::PointXYZ>::Ptr &);

//! Compute the time since "begin"
double durationMillis(clock_t &);

} // namespace segmentation
} // namespace bachelors_final_project

#endif // UTILS

