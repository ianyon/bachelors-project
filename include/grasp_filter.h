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
  moveit_msgs::AllowedCollisionMatrix getCollisionMatrix(moveit_msgs::GetPlanningScene scene_srv,
                                                         bool after);

  ros::Publisher display_publisher;
  ros::Publisher collision_obj_publisher;
  ros::Publisher attached_obj_publisher;

  ros::Publisher planning_scene_diff_publisher;

  ros::ServiceClient client_get_scene;

  tf::TransformListener transform_listener;

  tf::StampedTransform stamped_transform;
  BoundingBoxPtr bounding_box_;
  CloudPtr side_grasps_, top_grasps_;

  CloudPtr feasible_side_grasps_, feasible_top_grasps_;

  typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

  moveit::planning_interface::MoveGroup group_;

  //planning_scene::PlanningScene planning_scene;
  static const std::string GRASPABLE_OBJECT;
  static const std::string SUPPORT_TABLE;

  std::string kinect_frame_id_;

  CloudPtr table_world_coords_;

public:
  GraspFilter(ros::NodeHandle &handle,tf::TransformListener &tf_listener);

  void configure(std::string kinect_frame_id, BoundingBoxPtr &bounding_box, CloudPtr &table_plane);

  void tableCoords(CloudPtr &table_world_coords, BoundingBoxPtr &bounding_box, Point *min,
                   Point *max, Point *size);

  void addSupportTable(Point &table_min, Point &table_max, Point &size);

  void addCollisionObject(BoundingBoxPtr &bounding_box);

  void updateAllowedCollisionMatrix();

  geometry_msgs::Pose newPose(Eigen::Vector3f &pose);

  void filterGraspingPoses(CloudPtr side_grasps, CloudPtr top_grasps);

  void initializePublisher(ros::NodeHandle &handle);

  void visualizePlan(moveit::planning_interface::MoveGroup::Plan pose_plan);

  Cloud processGraspSamples(CloudPtr samples);

  bool processSample(Point &sample);

  CloudPtr getSideGrasps();

  CloudPtr getTopGrasps();

  tf::TransformListener &tf_listener_;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
