//
// Created by ian on 10/6/15.
//

#ifndef BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
#define BACHELORS_FINAL_PROJECT_GRASP_FILTER_H

#include <string>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>

#include "definitions.h"

namespace bachelors_final_project
{
namespace detection
{

class GraspFilter
{
public:
  GraspFilter();

  void filterGraspingPoses(PointCloudTPtr, PointCloudTPtr, std::string);

  ros::Publisher display_publisher;

  tf::TransformListener transform_listener;
  tf::StampedTransform stamped_transform;

  PointCloudTPtr side_grasps_, top_grasps_;

  static const std::string WORLD_FRAME;
  static const std::string ROBOT_BASE_FRAME;
  static const float WAIT_TRANSFORM_TIMEOUT;

  void initializePublisher(ros::NodeHandle &handle);

  void visualizePlan(moveit::planning_interface::MoveGroup::Plan pose_plan);
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
