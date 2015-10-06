//
// Created by ian on 10/6/15.
//

#include "grasp_filter.h"

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using std::string;

namespace bachelors_final_project
{

const std::string detection::GraspFilter::WORLD_FRAME = "/odom_combined";
const std::string detection::GraspFilter::ROBOT_BASE_FRAME = "/base_footprint";
const float detection::GraspFilter::WAIT_TRANSFORM_TIMEOUT = 1.0;

detection::GraspFilter::GraspFilter()
{

}

void detection::GraspFilter::filterGraspingPoses(PointCloudTPtr side_grasps, PointCloudTPtr top_grasps,
                                                 string kinect_frame_id)
{
  side_grasps_ = side_grasps;
  top_grasps_ = top_grasps;

  ros::Time transform_time = ros::Time(0);
  // Try to obtain the current transform
  try
  {
    ROS_INFO("Waiting available transform...");
    if (not transform_listener.waitForTransform(kinect_frame_id, ROBOT_BASE_FRAME, transform_time,
                                                ros::Duration(WAIT_TRANSFORM_TIMEOUT)))
    {
      ROS_ERROR("Failed to obtain transform in %gs", WAIT_TRANSFORM_TIMEOUT);
      return;
    }
    transform_listener.lookupTransform(kinect_frame_id, ROBOT_BASE_FRAME, ros::Time(0), stamped_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Excepción al obtener transformación: %s", ex.what());
    return;
  }

  //transform_listener.transformQuaternion(ROBOT_BASE_FRAME,normal_quat,normal_quat_baseframe);

  moveit::planning_interface::MoveGroup group("right_arm");
  // Para tratar con el mundo
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO("Frame de referencia: %s", group.getPlanningFrame().c_str());
  ROS_INFO("End Effector link: %s", group.getEndEffectorLink().c_str());

  // Define a goal pose
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = 0.5;
  goal_pose.position.y = -0.5;
  goal_pose.position.z = 1.1;
  goal_pose.orientation.w = 1;
  group.setPoseTarget(goal_pose);
  // Plan trayectory
  moveit::planning_interface::MoveGroup::Plan pose_plan;
  bool success = group.plan(pose_plan);
  ROS_INFO("Visualizando plan 1 (Pose Goal) %s", success ? "" : "FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(5.0);
  if (success) visualizePlan(pose_plan);
}

void detection::GraspFilter::visualizePlan(moveit::planning_interface::MoveGroup::Plan pose_plan)
{
  ROS_INFO("Visualizing plan again:");
  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = pose_plan.start_state_;
  display_trajectory.trajectory.push_back(pose_plan.trajectory_);
  display_publisher.publish(display_trajectory);
  sleep(5.0); // Para alcanzar a ver la trayectoria
}

void detection::GraspFilter::initializePublisher(ros::NodeHandle &handle)
{
  display_publisher = handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}
} // namespace bachelors_final_project
