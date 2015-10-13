//
// Created by ian on 10/6/15.
//

#include "grasp_filter.h"

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <shape_tools/solid_primitive_dims.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include "containers.h"

using std::string;

namespace bachelors_final_project
{

const std::string detection::GraspFilter::WORLD_FRAME = "/odom_combined";
const std::string detection::GraspFilter::ROBOT_BASE_FRAME = "/base_footprint";
const std::string detection::GraspFilter::GRASPABLE_OBJECT = "graspable object";

const float detection::GraspFilter::WAIT_TRANSFORM_TIMEOUT = 1.0;

detection::GraspFilter::GraspFilter()
{
  group_.reset(new moveit::planning_interface::MoveGroup("right_arm"));

  ROS_INFO("Frame de referencia: %s", group_->getPlanningFrame().c_str());
  ROS_INFO("End Effector link: %s", group_->getEndEffectorLink().c_str());
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

  processGraspSamples(side_grasps_);
  processGraspSamples(top_grasps_);
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

  collision_obj_publisher = handle.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  // Not sure really need this
  attached_obj_publisher = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
}


void detection::GraspFilter::processGraspSamples(PointCloudTPtr samples)
{
  // TODO select the group closer to the object

  for (int i = 0; i < samples->size(); ++i)
  {
    processSample(samples->points[i]);
  }
}

void detection::GraspFilter::processSample(PointT &sample)
{
  //TODO: Transform coordinates?
  // Define a goal pose
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = sample.x;
  goal_pose.position.y = sample.y;
  goal_pose.position.z = sample.z;
  goal_pose.orientation.w = 1;
  group_->setPoseTarget(goal_pose);

  group_->setPlanningTime(5.0); // 5s

  // Plan trayectory
  moveit::planning_interface::MoveGroup::Plan pose_plan;
  bool success = group_->plan(pose_plan);
  ROS_INFO("Visualizando plan 1 (Pose Goal) %s", success ? "" : "FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(5.0);
  //if (success) visualizePlan(pose_plan);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

/*
  planning_scene = new planning_scene::PlanningScene(robot.robotModel());
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 6: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");
*/
}

void detection::GraspFilter::addCollisionObject(BoundingBox &bounding_box)
{
  moveit_msgs::CollisionObject co;
  co.id = GRASPABLE_OBJECT;
  co.header.stamp = ros::Time::now();
  // TODO check
  co.header.frame_id = group_->getPlanningFrame();

  // First remove it
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  collision_obj_publisher.publish(co);

  //Is this needed
  moveit_msgs::AttachedCollisionObject aco;
  aco.object = co;
  attached_obj_publisher.publish(aco);

  co.operation = moveit_msgs::CollisionObject::ADD;

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = bounding_box.getXLength();
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = bounding_box.getYLength();
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = bounding_box.getZLength();
  collision_object.primitives.push_back(primitive);

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = bounding_box.mean_diag[0];
  box_pose.position.y = bounding_box.mean_diag[1];
  box_pose.position.z = bounding_box.mean_diag[2];
  collision_object.primitive_poses.push_back(box_pose);

  collision_obj_publisher.publish(co);
}

} // namespace bachelors_final_project
