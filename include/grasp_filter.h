//
// Created by ian on 10/6/15.
//

#ifndef BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
#define BACHELORS_FINAL_PROJECT_GRASP_FILTER_H

#include <string>
#include <ros/node_handle.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/ModelCoefficients.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/GetPlanningScene.h>

#include "definitions.h"
#include "containers.h"

namespace bachelors_final_project
{
namespace detection
{

class GraspFilter
{
  moveit_msgs::AllowedCollisionMatrix getCollisionMatrix(moveit_msgs::GetPlanningScene scene_srv);
  ros::Publisher display_publisher;
  ros::Publisher collision_obj_publisher;
  ros::Publisher attached_obj_publisher;

  ros::Publisher planning_scene_diff_publisher;

  ros::ServiceClient client_get_scene;

  tf::TransformListener transform_listener;

  tf::StampedTransform stamped_transform;
  BoundingBoxPtr bounding_box_;
  PointCloudTPtr side_grasps_, top_grasps_;

  PointCloudTPtr feasible_side_grasps_, feasible_top_grasps_;

  typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

  MoveGroupPtr group_;

  //planning_scene::PlanningScene planning_scene;
  static const std::string GRASPABLE_OBJECT;
  static const std::string SUPPORT_TABLE;

  std::string kinect_frame_id_;

  pcl::ModelCoefficientsPtr table_plane_;

public:

  GraspFilter();

  void configure(std::string kinect_frame_id, BoundingBoxPtr &bounding_box, pcl::ModelCoefficientsPtr &table_plane_);

  void filterGraspingPoses(PointCloudTPtr side_grasps, PointCloudTPtr top_grasps);

  void initializePublisher(ros::NodeHandle &handle);

  void visualizePlan(moveit::planning_interface::MoveGroup::Plan pose_plan);

  PointCloudT processGraspSamples(PointCloudTPtr samples);

  bool processSample(PointT &sample);

  void addSupportTable(pcl::ModelCoefficientsPtr &table_plane, BoundingBoxPtr &bounding_box);

  void addCollisionObject(BoundingBoxPtr &bounding_box);

  void updateAllowedCollisionMatrix();

  PointCloudTPtr getSideGrasps();

  PointCloudTPtr getTopGrasps();
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
