//
// Created by ian on 9/29/15.
//

#ifndef BACHELORS_FINAL_PROJECT_DEFINITIONS_H
#define BACHELORS_FINAL_PROJECT_DEFINITIONS_H

#include <pcl/point_types.h>

#include <pcl/point_cloud.h>
#include <ros/ros.h>
namespace bachelors_final_project
{
static const std::string WORLD_FRAME = "/odom_combined";
static const std::string FOOTPRINT_FRAME = "/base_footprint";
static const std::string KINECT_TOPIC = "/head_mount_kinect/depth/points";
static const float TF_TIMEOUT = 2.0;
static const ros::Duration TF_DURATION(TF_TIMEOUT);

enum FailedLevel { NORMALS, PLANE, OVER_TABLE, CLUSTERS };

typedef pcl::PointXYZ Point;
typedef pcl::Normal Normal;

typedef pcl::PointCloud <pcl::PointXYZ> Cloud;
typedef pcl::PointCloud <pcl::PointXYZ>::Ptr CloudPtr;
typedef pcl::PointCloud <pcl::PointXYZ>::ConstPtr CloudConstPtr;

typedef pcl::PointCloud <pcl::Normal> CloudNormal;
typedef pcl::PointCloud <pcl::Normal>::Ptr CloudNormalPtr;

} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_DEFINITIONS_H
