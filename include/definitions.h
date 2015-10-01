//
// Created by ian on 9/29/15.
//

#ifndef BACHELORS_FINAL_PROJECT_DEFINITIONS_H
#define BACHELORS_FINAL_PROJECT_DEFINITIONS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace bachelors_final_project
{
typedef pcl::PointXYZ PointT;
typedef pcl::Normal Normal;

typedef pcl::PointCloud <pcl::PointXYZ> PointCloudT;
typedef pcl::PointCloud <pcl::PointXYZ>::Ptr PointCloudTPtr;
typedef pcl::PointCloud <pcl::PointXYZ>::ConstPtr PointCloudTConstPtr;

typedef pcl::PointCloud <pcl::Normal> PointCloudNormal;
typedef pcl::PointCloud <pcl::Normal>::Ptr PointCloudNormalPtr;
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_DEFINITIONS_H
