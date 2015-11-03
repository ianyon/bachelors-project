#ifndef UTILS
#define UTILS

#include <pcl/PointIndices.h>

#include <tf/transform_listener.h>
#include <pcl/filters/project_inliers.h>

#include "definitions.h"

namespace bachelors_final_project
{

bool cloudSizeComparator(CloudPtr i, CloudPtr j)
{ return (i->size() > j->size()); }

int getNBiggerIndex(size_t size, int n);

void callable(pcl::visualization::PCLVisualizer &viz);

pcl::PointXYZRGB newPointXYZRGB(Point p, uint8_t r, uint8_t g, uint8_t b);

Eigen::Vector3f newVector3f(Point p);
Point newPoint(Eigen::Vector3f v);

//! Compute the time since "begin"
double durationMillis(clock_t &);

void setProperties(const CloudPtr &coppied_cloud, CloudPtr &cloud_out, int width, int height);

Eigen::Vector3d castVector3d(Eigen::Vector3f v);

bool transformPoint(const std::string &init_frame, const std::string &final_frame, const Eigen::Vector3f &point_in,
                    Point &point_out, uint64_t micro_sec_time, tf::TransformListener &tf_listener);

bool transformPose(const std::string &init_frame, const std::string &final_frame,
                   geometry_msgs::PoseStamped &pose_in, geometry_msgs::PoseStamped &pose_out,
                   uint64_t micro_sec_time, tf::TransformListener &tf_listener);

bool transformPointCloud(const std::string &init_frame, const std::string &final_frame, const CloudPtr &cloud_in,
                         const CloudPtr &cloud_out, const uint64_t micro_sec_time,
                         tf::TransformListener &tf_listener);

void projectOnPlane(const CloudPtr &cloud, const pcl::ModelCoefficientsPtr &table,
                    const pcl::PointIndices::Ptr &indices, CloudPtr &projected_cloud);

void projectOnPlane(const CloudPtr &cloud, const pcl::ModelCoefficientsPtr &table, CloudPtr &projected_cloud);

pcl::ProjectInliers<Point> getProjector(const CloudPtr &sensor_cloud,
                                        const pcl::ModelCoefficientsPtr &table);

Point fourToPoint(Eigen::Vector4f &vector);
Point threeToPoint(Eigen::Vector3f &vector);

int selectChoice(std::string message);

float selectChoiceFloat(std::string message);

void theThing();

} // namespace bachelors_final_project

#endif // UTILS

