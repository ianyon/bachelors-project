#ifndef UTILS
#define UTILS

#include <pcl/PointIndices.h>

#include "definitions.h"

namespace bachelors_final_project
{

//! Publish the message.
void publish(const ros::Publisher &, const PointCloudTPtr&);

//! Compute the time since "begin"
double durationMillis(clock_t &);

void setProperties(const PointCloudTPtr &coppied_cloud, PointCloudTPtr &cloud_out, int width, int height);

void extractPointCloud(const PointCloudTPtr &input, pcl::PointIndices::Ptr &inliers, PointCloudTPtr &output,
                       bool extract_negative_set = false);

void extractPointCloud(const PointCloudNormalPtr &input, pcl::PointIndices::Ptr &inliers, PointCloudNormalPtr &output,
                       bool extract_negative_set = false);

bool transformPoint(const std::string &init_frame, const std::string &final_frame, const PointT &point_in,
                    PointT &point_out, uint64_t micro_sec_time);

bool transformPointCloud(const std::string &init_frame, const std::string &final_frame, const PointCloudTPtr &cloud_in,
                         const PointCloudTPtr &cloud_out, const uint64_t micro_sec_time);


} // namespace bachelors_final_project

#endif // UTILS

