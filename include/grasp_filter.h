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


#include <pcl/visualization/cloud_viewer.h>

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
  ros::Publisher test_pub,test_pub2;
  CloudPtr test_cloud;
  CloudPtr test_cloud2;

  ros::ServiceClient client_get_scene;

  tf::TransformListener &tf_listener_;
  tf::StampedTransform stamped_transform;

  CloudPtr side_grasps_, top_grasps_;
  CloudPtr feasible_side_grasps_, feasible_top_grasps_;

  typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

  moveit::planning_interface::MoveGroup group_;

  //planning_scene::PlanningScene planning_scene;
  static const std::string GRASPABLE_OBJECT;
  static const std::string SUPPORT_TABLE;

  std::string kinect_frame_id_;

public:
  GraspFilter(ros::NodeHandle &handle,tf::TransformListener &tf_listener);

  void configure(std::string kinect_frame_id, BoundingBoxPtr &obj_bounding_box, BoundingBoxPtr &table_bounding_box);

  void tableCoords(CloudPtr &table_world_coords, BoundingBoxPtr &bounding_box, Point *pose,
                   Point *size);

  void addSupportTable(Point &pose, Eigen::Vector3f &size);

  void addCollisionObject(Point &pose_pt, Eigen::Vector3f &size);

  void updateAllowedCollisionMatrix();

  geometry_msgs::Pose newPose(Eigen::Vector3f pose);

  void filterGraspingPoses(CloudPtr side_grasps, CloudPtr top_grasps);

  void initializePublisher(ros::NodeHandle &handle);

  void visualizePlan(moveit::planning_interface::MoveGroup::Plan pose_plan);

  Cloud processGraspSamples(CloudPtr samples);

  bool processSample(Point &sample);

  CloudPtr getSideGrasps();

  CloudPtr getTopGrasps();
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
