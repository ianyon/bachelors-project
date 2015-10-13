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
  ros::Publisher collision_obj_publisher;
  ros::Publisher attached_obj_publisher;

  tf::TransformListener transform_listener;
  tf::StampedTransform stamped_transform;

  PointCloudTPtr side_grasps_, top_grasps_;

  typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

  MoveGroupPtr group_;

  //planning_scene::PlanningScene planning_scene;

  static const std::string WORLD_FRAME;
  static const std::string ROBOT_BASE_FRAME;
  static const float WAIT_TRANSFORM_TIMEOUT;

  void initializePublisher(ros::NodeHandle &handle);

  void visualizePlan(moveit::planning_interface::MoveGroup::Plan pose_plan);

  void processGraspSamples(PointCloudTPtr samples);

  void processSample(PointT &sample);

  void addCollisionObject();
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
