//
// Created by ian on 10/6/15.
//

#include "grasp_filter.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <shape_tools/solid_primitive_dims.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/GetPlanningScene.h>

using std::string;

namespace bachelors_final_project
{

const std::string detection::GraspFilter::WORLD_FRAME = "/odom_combined";
const std::string detection::GraspFilter::ROBOT_BASE_FRAME = "/base_footprint";
const std::string detection::GraspFilter::GRASPABLE_OBJECT = "graspable object";
const std::string detection::GraspFilter::SUPPORT_TABLE = "table";

const float detection::GraspFilter::WAIT_TRANSFORM_TIMEOUT = 1.0;

detection::GraspFilter::GraspFilter()
{
  group_.reset(new moveit::planning_interface::MoveGroup("right_arm"));

  ROS_INFO("Frame de referencia: %s", group_->getPlanningFrame().c_str());
  ROS_INFO("End Effector link: %s", group_->getEndEffectorLink().c_str());
}

void detection::GraspFilter::initializePublisher(ros::NodeHandle &handle)
{
  display_publisher = handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  collision_obj_publisher = handle.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  // Not sure really need this
  attached_obj_publisher = handle.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  client_get_scene = handle.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  planning_scene_diff_publisher = handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
}

void detection::GraspFilter::configure(string kinect_frame_id, BoundingBoxPtr &bounding_box,
                                       pcl::ModelCoefficientsPtr &table_plane)
{
  kinect_frame_id_ = kinect_frame_id;
  bounding_box_ = bounding_box;
  table_plane_ = table_plane;

  ros::Time transform_time = ros::Time(0);
  // Try to obtain the current transform
  try
  {
    ROS_INFO("Waiting available transform...");
    if (not transform_listener.waitForTransform(kinect_frame_id_, ROBOT_BASE_FRAME, transform_time,
                                                ros::Duration(WAIT_TRANSFORM_TIMEOUT)))
    {
      ROS_ERROR("Failed to obtain transform in %gs", WAIT_TRANSFORM_TIMEOUT);
      return;
    }
    transform_listener.lookupTransform(kinect_frame_id_, ROBOT_BASE_FRAME, ros::Time(0), stamped_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Excepción al obtener transformación: %s", ex.what());
    return;
  }

  //transform_listener.transformQuaternion(ROBOT_BASE_FRAME,normal_quat,normal_quat_baseframe);

  // Add the table to supress collisions of the object with it
  addSupportTable(table_plane_, bounding_box_);

  // Add a box to supress collisions with the object
  addCollisionObject(bounding_box_);
  updateAllowedCollisionMatrix();
}

void detection::GraspFilter::updateAllowedCollisionMatrix()
{
  moveit_msgs::GetPlanningScene scene_srv;
  scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

  if(!client_get_scene.call(scene_srv))
  {
    ROS_WARN("Failed to call service /get_planning_scene");
  }
  else
  {
    moveit_msgs::AllowedCollisionMatrix &currentACM = getCollisionMatrix(scene_srv);

    currentACM.entry_names.push_back(GRASPABLE_OBJECT);
    moveit_msgs::AllowedCollisionEntry entry;
    entry.enabled.resize(currentACM.entry_names.size());

    // Add new row to allowed collision matrix
    //TODO: Make objects allowed to collision only with gripper
    for(int i = 0; i < entry.enabled.size(); i++)
    {
      entry.enabled[i] = true;
    }

    currentACM.entry_values.push_back(entry);

    // Extend the last column of the matrix
    for(int i = 0; i < currentACM.entry_values.size(); i++)
      currentACM.entry_values[i].enabled.push_back(true);

    moveit_msgs::PlanningScene newSceneDiff;
    newSceneDiff.is_diff = true;
    newSceneDiff.allowed_collision_matrix = currentACM;
    planning_scene_diff_publisher.publish(newSceneDiff);
  }

  // To check that the matrix was properly updated
  if(!client_get_scene.call(scene_srv))
  {
    ROS_WARN("Failed to call service /get_planning_scene");
  }
  else
  {
    getCollisionMatrix(scene_srv);
  }
}

moveit_msgs::AllowedCollisionMatrix& detection::GraspFilter::getCollisionMatrix(moveit_msgs::GetPlanningScene scene_srv)
{
  ROS_INFO_STREAM("Modified scene!");
  moveit_msgs::PlanningScene currentScene = scene_srv.response.scene;
  moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;

  ROS_ERROR_STREAM("size of acm_entry_names after " << currentACM.entry_names.size());
  ROS_ERROR_STREAM("size of acm_entry_values after " << currentACM.entry_values.size());
  ROS_ERROR_STREAM("size of acm_entry_values[0].entries after " << currentACM.entry_values[0].enabled.size());
  return currentACM;
}

void detection::GraspFilter::filterGraspingPoses(PointCloudTPtr side_grasps, PointCloudTPtr top_grasps)
{
  side_grasps_ = side_grasps;
  top_grasps_ = top_grasps;

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
}

void detection::GraspFilter::addSupportTable(pcl::ModelCoefficientsPtr &table_plane, BoundingBoxPtr &bounding_box)
{
  moveit_msgs::CollisionObject co;
  co.id = SUPPORT_TABLE;
  co.header.stamp = ros::Time::now();
  // TODO check
  co.header.frame_id = group_->getPlanningFrame();

  // First remove it
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  collision_obj_publisher.publish(co);

  co.operation = moveit_msgs::CollisionObject::ADD;

  /* Define a plane to add to the world. */
  shape_msgs::Plane plane;
  plane.coef[0] = table_plane->values[0];
  plane.coef[0] = table_plane->values[0];
  plane.coef[0] = table_plane->values[0];
  plane.coef[0] = table_plane->values[0];

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = bounding_box->mean_diag[0];
  box_pose.position.y = bounding_box->mean_diag[1];
  box_pose.position.z = bounding_box->mean_diag[2];
  co.primitive_poses.push_back(box_pose);

  collision_obj_publisher.publish(co);
}

void detection::GraspFilter::addCollisionObject(BoundingBoxPtr &bounding_box)
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
  // TODO: Axes may not be right
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = bounding_box->getXLength();
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = bounding_box->getYLength();
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = bounding_box->heigth_3D;
  co.primitives.push_back(primitive);

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = bounding_box->mean_diag[0];
  box_pose.position.y = bounding_box->mean_diag[1];
  box_pose.position.z = bounding_box->mean_diag[2];
  co.primitive_poses.push_back(box_pose);

  collision_obj_publisher.publish(co);
}

} // namespace bachelors_final_project
