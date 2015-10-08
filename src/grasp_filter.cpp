//
// Created by ian on 10/6/15.
//

#include "grasp_filter.h"

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_interface/planning_interface.h>
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
}

void detection::GraspFilter::processGraspSamples(PointCloudTPtr samples)
{
  // TODO select the group closer to the object
  group_.reset(new moveit::planning_interface::MoveGroup("right_arm"));

  ROS_INFO("Frame de referencia: %s", group_->getPlanningFrame().c_str());
  ROS_INFO("End Effector link: %s", group_->getEndEffectorLink().c_str());

  /*roslaunch pr2_gazebo pr2_empty_world.lauch   // repeat this until no crash
roslaunch pr2_moveit_config move_group.launch
roslaunch pr2_moveit_config moveit_rviz.launch
run  ./devel/lib/pr2_moveit_tutorials/pick_place_tutorial*/

  robot_state::RobotState start_state(*group_->getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  const robot_state::JointModelGroup *joint_model_group =
      start_state.getJointModelGroup(group_->getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group_->setStartState(start_state);

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group_->setPathConstraints(test_constraints);

  /*BOOST_FOREACH(PointT sample, samples)
  {
    processSample(sample);
  }*/

  for (int i = 0; i < samples->size(); ++i)
  {
    processSample(samples->points[i]);
  }
  group_->clearPathConstraints();
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
  if (success) visualizePlan(pose_plan);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  /*
   * rosservice list
    <param name="capabilities" value="move_group/MoveGroupCartesianPathService
                      move_group/MoveGroupExecuteService
                      move_group/MoveGroupKinematicsService
                      move_group/MoveGroupMoveAction
                      move_group/MoveGroupPickPlaceAction
                      move_group/MoveGroupPlanService
                      move_group/MoveGroupQueryPlannersService
                      move_group/MoveGroupStateValidationService
                      move_group/MoveGroupGetPlanningSceneService
                      " />
*/

  /*
   *  req.workspace_parameters.header.frame_id = base_link;
        req.workspace_parameters.max_corner.x = 100;
        req.workspace_parameters.max_corner.y = 100;
        req.workspace_parameters.max_corner.z = 100;
        req.workspace_parameters.min_corner.x = -100;
        req.workspace_parameters.min_corner.y = -100;
        req.workspace_parameters.min_corner.z = -100;
        req.num_planning_attempts = 10;
        moveit_msgs::Constraints path_const =
                kinematic_constraints::constructGoalConstraints(tip_name,
                        pose_path, 10, 10);
        path_const.orientation_constraints.clear();

        bool satisfied;
        satisfied = psm.getPlanningScene()->isStateConstrained(
                psm.getPlanningScene()->getCurrentState(), path_const, true); //RETURN TRUE
        ROS_INFO_STREAM("Current scene status w.r.t. path_const:" << satisfied);

        if (use_path_const) { //TRUE
            req.path_constraints.position_constraints.push_back(
                    path_const.position_constraints[0]);
        }

        planning_pipeline->generatePlan(psm.getPlanningScene(), req, res);
   * */
  /*collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  robot_state::RobotState copied_state = planning_scene.getCurrentState();
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);*/
  ROS_INFO_STREAM("Test 6: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");

}

void detection::GraspFilter::addCollisionObject()
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group_->getPlanningFrame();

/* The id of the object is used to identify it. */
  collision_object.id = "box1";

/* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

/* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.6;
  box_pose.position.y = -0.4;
  box_pose.position.z = 1.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO("Add an object into the world");
  // Para tratar con el mundo
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.addCollisionObjects(collision_objects);

/* Sleep so we have time to see the object in RViz */
  sleep(2.0);
}

} // namespace bachelors_final_project
