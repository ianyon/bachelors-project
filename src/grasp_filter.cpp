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
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/foreach.hpp>

#include "bounding_box.h"

using std::string;
using namespace moveit_msgs;

namespace bachelors_final_project
{

const std::string detection::GraspFilter::GRASPABLE_OBJECT = "graspable object";
const std::string detection::GraspFilter::SUPPORT_TABLE = "table";

detection::GraspFilter::GraspFilter(ros::NodeHandle &handle) :
    group_(new moveit::planning_interface::MoveGroup("right_arm")),
    display_publisher(handle.advertise<DisplayTrajectory>("/move_group/display_planned_path", 1, true)),
    collision_obj_publisher(handle.advertise<CollisionObject>("collision_object", 10)),
    // Not sure really need this
    attached_obj_publisher(handle.advertise<AttachedCollisionObject>("attached_collision_object", 10)),
    client_get_scene(handle.serviceClient<GetPlanningScene>("/get_planning_scene")),
    planning_scene_diff_publisher(handle.advertise<PlanningScene>("monitored_planning_scene", 1))
{
  ROS_INFO("Frame de referencia: %s", group_->getPlanningFrame().c_str());
  ROS_INFO("End Effector link: %s", group_->getEndEffectorLink().c_str());
}

void detection::GraspFilter::configure(string kinect_frame_id, BoundingBoxPtr &bounding_box,
                                       pcl::ModelCoefficientsPtr &table_plane)
{
  kinect_frame_id_ = kinect_frame_id;
  bounding_box_ = bounding_box;
  table_plane_ = table_plane;

  // Add the table to supress collisions of the object with it
  addSupportTable(table_plane_, bounding_box_);

  // Add a box to supress collisions with the object
  addCollisionObject(bounding_box_);
  updateAllowedCollisionMatrix();
}

void detection::GraspFilter::updateAllowedCollisionMatrix()
{
  GetPlanningScene scene_srv;
  scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

  if (!client_get_scene.call(scene_srv))
  {
    ROS_WARN("Failed to call service /get_planning_scene");
  }
  else
  {
    AllowedCollisionMatrix currentACM = getCollisionMatrix(scene_srv, false);

    currentACM.entry_names.push_back(GRASPABLE_OBJECT);
    AllowedCollisionEntry entry;
    entry.enabled.resize(currentACM.entry_names.size());

    // Add new row to allowed collision matrix
    //TODO: Make objects allowed to collision only with gripper
    for (int i = 0; i < entry.enabled.size(); i++)
    {
      entry.enabled[i] = true;
    }

    currentACM.entry_values.push_back(entry);

    // Extend the last column of the matrix
    for (int i = 0; i < currentACM.entry_values.size(); i++)
      currentACM.entry_values[i].enabled.push_back(true);

    PlanningScene newSceneDiff;
    newSceneDiff.is_diff = true;
    newSceneDiff.allowed_collision_matrix = currentACM;
    planning_scene_diff_publisher.publish(newSceneDiff);
  }

  // To check that the matrix was properly updated
  if (!client_get_scene.call(scene_srv))
  {
    ROS_WARN("Failed to call service /get_planning_scene");
  }
  else
  {
    getCollisionMatrix(scene_srv, true);
  }
}

AllowedCollisionMatrix detection::GraspFilter::getCollisionMatrix(GetPlanningScene scene_srv,
                                                                  bool after)
{
  ROS_INFO("Modified scene!");
  AllowedCollisionMatrix currentACM = scene_srv.response.scene.allowed_collision_matrix;
  std::string case_ = after ? "after " : "before ";
  ROS_WARN("Size of acm_entry %s {names:%lu, values:%lu, entry:%lu}", case_.c_str(), currentACM.entry_names.size(),
           currentACM.entry_values.size(), currentACM.entry_values[0].enabled.size());
  return currentACM;
}

void detection::GraspFilter::filterGraspingPoses(CloudPtr side_grasps, CloudPtr top_grasps)
{
  side_grasps_ = side_grasps;
  top_grasps_ = top_grasps;

  //feasible_side_grasps_.reset(new Cloud(processGraspSamples(side_grasps_)));
  //feasible_top_grasps_.reset(new Cloud(processGraspSamples(top_grasps_)));
}

void detection::GraspFilter::visualizePlan(moveit::planning_interface::MoveGroup::Plan pose_plan)
{
  ROS_INFO("Visualizing plan again:");
  DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = pose_plan.start_state_;
  display_trajectory.trajectory.push_back(pose_plan.trajectory_);
  display_publisher.publish(display_trajectory);
  sleep(5.0); // Para alcanzar a ver la trayectoria
}


Cloud detection::GraspFilter::processGraspSamples(CloudPtr samples)
{
  // TODO select the group closer to the object
  Cloud feasible_grasps;

  BOOST_FOREACH(Point point, samples->points)
        {
          if (processSample(point))
            feasible_grasps.push_back(point);
        }
  return feasible_grasps;
}

bool detection::GraspFilter::processSample(Point &sample)
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
  //sleep(5.0);
  //if (success) visualizePlan(pose_plan);
  return success;
}

void detection::GraspFilter::addSupportTable(pcl::ModelCoefficientsPtr &table_plane, BoundingBoxPtr &bounding_box)
{
  CollisionObject co;
  co.id = SUPPORT_TABLE;
  co.header.stamp = ros::Time::now();
  // TODO check
  co.header.frame_id = group_->getPlanningFrame();

  // First remove it
  co.operation = CollisionObject::REMOVE;
  collision_obj_publisher.publish(co);

  co.operation = CollisionObject::ADD;

  /* Define a plane to add to the world. */
  shape_msgs::Plane plane;
  plane.coef[0] = table_plane->values[0];
  plane.coef[0] = table_plane->values[0];
  plane.coef[0] = table_plane->values[0];
  plane.coef[0] = table_plane->values[0];

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = bounding_box->worldCoordsBoundingBoxPose()[0];
  box_pose.position.y = bounding_box->worldCoordsBoundingBoxPose()[1];
  box_pose.position.z = bounding_box->worldCoordsBoundingBoxPose()[2];
  co.primitive_poses.push_back(box_pose);

  collision_obj_publisher.publish(co);
}

void detection::GraspFilter::addCollisionObject(BoundingBoxPtr &bounding_box)
{
  CollisionObject co;
  co.id = GRASPABLE_OBJECT;
  co.header.stamp = ros::Time::now();
  // TODO check
  co.header.frame_id = group_->getPlanningFrame();

  // First remove it
  co.operation = CollisionObject::REMOVE;
  collision_obj_publisher.publish(co);

  //Is this needed
  AttachedCollisionObject aco;
  aco.object = co;
  attached_obj_publisher.publish(aco);

  co.operation = CollisionObject::ADD;

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  // TODO: Axes may not be right
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = bounding_box->getXLengthWorldCoords();
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = bounding_box->getYLengthWorldCoords();
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = bounding_box->getZLengthWorldCoords();
  co.primitives.push_back(primitive);

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = bounding_box->worldCoordsBoundingBoxPose()[0];
  box_pose.position.y = bounding_box->worldCoordsBoundingBoxPose()[1];
  box_pose.position.z = bounding_box->worldCoordsBoundingBoxPose()[2];
  co.primitive_poses.push_back(box_pose);

  collision_obj_publisher.publish(co);
}

CloudPtr detection::GraspFilter::getSideGrasps()
{
  return feasible_side_grasps_;
}

CloudPtr detection::GraspFilter::getTopGrasps()
{
  return feasible_top_grasps_;
}
} // namespace bachelors_final_project
