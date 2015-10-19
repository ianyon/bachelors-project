#ifndef UTILS
#define UTILS

#include <pcl/PointIndices.h>

#include <tf/transform_listener.h>

#include "definitions.h"

namespace bachelors_final_project
{

//! Publish the message.
void publish(const ros::Publisher &, const PointCloudPtr &);

//! Compute the time since "begin"
double durationMillis(clock_t &);

void setProperties(const PointCloudPtr &coppied_cloud, PointCloudPtr &cloud_out, int width, int height);

void pointCloudFromIndices(const PointCloudPtr &input, pcl::PointIndicesPtr &inliers, PointCloudPtr &output,
                           bool extract_negative_set = false);

void normalPointCloudFromIndices(const PointCloudNormalPtr &input, pcl::PointIndicesPtr &inliers,
                                 PointCloudNormalPtr &output,
                                 bool extract_negative_set = false);

bool transformPoint(const std::string &init_frame, const std::string &final_frame, const tf::Vector3 &point_in,
                    Point &point_out, uint64_t micro_sec_time, tf::TransformListener &tf_listener);

bool transformPointCloud(const std::string &init_frame, const std::string &final_frame, const PointCloudPtr &cloud_in,
                         const PointCloudPtr &cloud_out, const uint64_t micro_sec_time,
                         tf::TransformListener &tf_listener);


} // namespace bachelors_final_project

#endif // UTILS

