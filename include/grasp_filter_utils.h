//
// Created by yoko on 27-10-15.
//

#ifndef PROJECT_GRASP_FILTER_UTILS_H
#define PROJECT_GRASP_FILTER_UTILS_H

#include <ros/publisher.h>

#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

#include "definitions.h"

namespace bachelors_final_project
{
namespace detection
{

moveit_msgs::CollisionObject getCollisionObjUpdated(ros::Publisher pub, const std::string &id,
                                                    bool detach, ros::Publisher attached_pub);

shape_msgs::SolidPrimitive constructPrimitive(float sizeX, float sizeY, float sizeZ);

geometry_msgs::Pose newPose(Eigen::Vector3f pose);

geometry_msgs::Pose newPose(Point pose);

geometry_msgs::Pose newPose(Point pose, Eigen::Quaternionf rotation);

} // namespace detection
} // namespace bachelors_final_project


#endif //PROJECT_GRASP_FILTER_UTILS_H
