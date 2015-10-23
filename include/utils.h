#ifndef UTILS
#define UTILS

#include <pcl/PointIndices.h>

#include <tf/transform_listener.h>
#include <pcl/filters/project_inliers.h>

#include "definitions.h"

namespace bachelors_final_project
{

bool cloudSize(CloudPtr i, CloudPtr j)
{ return (i->size() < j->size()); }

//! Publish the message.
void publish(const ros::Publisher &, const CloudPtr &);

//! Compute the time since "begin"
double durationMillis(clock_t &);

void setProperties(const CloudPtr &coppied_cloud, CloudPtr &cloud_out, int width, int height);

bool transformPoint(const std::string &init_frame, const std::string &final_frame, const tf::Vector3 &point_in,
                    Point &point_out, uint64_t micro_sec_time, tf::TransformListener &tf_listener);

bool transformPointCloud(const std::string &init_frame, const std::string &final_frame, const CloudPtr &cloud_in,
                         const CloudPtr &cloud_out, const uint64_t micro_sec_time,
                         tf::TransformListener &tf_listener);

void projectOnPlane(const CloudPtr &cloud, const pcl::ModelCoefficientsPtr &table,
                    const pcl::PointIndices::Ptr &indices, CloudPtr &projected_cloud);

void projectOnPlane(const CloudPtr &cloud, const pcl::ModelCoefficientsPtr &table, CloudPtr &projected_cloud);

pcl::ProjectInliers<Point> getProjector(const CloudPtr &sensor_cloud,
                                        const pcl::ModelCoefficientsPtr &table);
} // namespace bachelors_final_project

#endif // UTILS

