//
// Created by ian on 10/6/15.
//

#include "grasp_filter.h"

#include <ros/ros.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_matrix.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "utils.h"
#include "bounding_box.h"
#include "grasp_filter_utils.h"

using std::string;
using namespace moveit_msgs;
using shape_msgs::SolidPrimitive;
using shape_msgs::Plane;

namespace bachelors_final_project
{

const std::string detection::GraspFilter::GRASPABLE_OBJECT = "graspable object";
const std::string detection::GraspFilter::SUPPORT_TABLE = "table";

detection::GraspFilter::GraspFilter(ros::NodeHandle &nh, tf::TransformListener &tf_listener) :
    group_("right_arm"),
    display_pub(nh.advertise<DisplayTrajectory>("/move_group/display_planned_path", 1, true)),
    collision_obj_pub(nh.advertise<CollisionObject>("collision_object", 10)),
    attached_obj_pub(nh.advertise<AttachedCollisionObject>("attached_collision_object", 10)),
    client_get_scene(nh.serviceClient<GetPlanningScene>("/get_planning_scene")),
    planning_scene_diff_pub(nh.advertise<PlanningScene>("planning_scene", 1)),
    test_pub(nh.advertise<Cloud>("test_cloud", 1)),
    test_pub2(nh.advertise<Cloud>("test_cloud2", 1)),
    tf_listener_(tf_listener)
{
  ROS_INFO("Frame de referencia: %s", group_.getPlanningFrame().c_str());
  ROS_INFO("End Effector link: %s", group_.getEndEffectorLink().c_str());
}


void detection::GraspFilter::configure(string kinect_frame_id, BoundingBoxPtr &obj_bounding_box,
                                       BoundingBoxPtr &table_bounding_box)
{
  kinect_frame_id_ = kinect_frame_id;

  test_cloud.reset(new Cloud);
  test_cloud->header.frame_id = kinect_frame_id;
  test_cloud2.reset(new Cloud);
  test_cloud2->header.frame_id = FOOTPRINT_FRAME;

  Point table_position = table_bounding_box->computePosition2DRobotFrame(tf_listener_);
  Eigen::Vector3f table_size = table_bounding_box->getSizeWithExternHeight(table_position.z);

  //table_bounding_box->visualizeData();
  //tableCoords(table_world_coords_, table_bounding_box, &table_position, &table_size);
  test_cloud2->push_back(table_position);
  //test_cloud2->push_back(table_size);
  test_pub.publish(test_cloud);
  test_pub2.publish(test_cloud2);
  CloudPtr cloud(test_cloud);
  *cloud += *test_cloud2;

  /*pcl::visualization::CloudViewer viewer("asd");
  viewer.showCloud(cloud);
  while(!viewer.wasStopped())
    ros::WallDuration(0.5).sleep();

  viewer.runOnVisualizationThreadOnce(callable);*/
  // Add the table to supress collisions of the object with it
  addSupportTable(table_position, table_size);

  // Add a box to supress collisions with the object
  //Point obj_pose = obj_bounding_box->computePosition2DRobotFrame(tf_listener_);
  Eigen::Vector3f obj_size = obj_bounding_box->getSize3D();
  //obj_size[2] = (float) obj_bounding_box->getHeigth();
  geometry_msgs::Pose obj_pose = obj_bounding_box->computePose3DRobotFrame(tf_listener_);
  addCollisionObject(obj_pose, obj_size, obj_bounding_box->OBJ_FRAME);
  //updateAllowedCollisionMatrix();
}

void detection::GraspFilter::addSupportTable(Point &pose_pt, Eigen::Vector3f &size)
{
  CollisionObject co = getCollisionObjUpdated(collision_obj_pub, SUPPORT_TABLE, false, attached_obj_pub,
                                              FOOTPRINT_FRAME);

  /* Define a plane as a box to add to the world. */
  co.primitives.push_back(constructPrimitive(size[0] * 0.9f, size[1] * 0.9f, size[2]));
  // A pose for the plane (specified relative to frame_id).
  // We need to divide z by two because we want the table to ocuppy all the space below the real table
  // 0.2796  is the border of the robot
  Eigen::Vector3f pose(fmax(pose_pt.x, 0.2796 + size[0] / 2), pose_pt.y, pose_pt.z * 0.5);
  co.primitive_poses.push_back(newPose(pose));

  collision_obj_pub.publish(co);
}

void detection::GraspFilter::addCollisionObject(geometry_msgs::Pose &pose_pt, Eigen::Vector3f &size,
                                                const std::string &frame)
{
  CollisionObject co = getCollisionObjUpdated(collision_obj_pub, GRASPABLE_OBJECT, true, attached_obj_pub,
                                              frame);
  /* Define a box to add to the world. */
  co.primitives.push_back(constructPrimitive(size[0], size[1], size[2]));
  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose pose;
  pose.orientation.w=1.0;
  co.primitive_poses.push_back(pose);
  collision_obj_pub.publish(co);
}


/*
void detection::GraspFilter::updateAllowedCollisionMatrix()
{
  GetPlanningScene scene_srv;
  scene_srv.request.components.components = scene_srv.request.components.ALLOWED_COLLISION_MATRIX;

  if (!client_get_scene.call(scene_srv))
  {
    ROS_WARN("Failed to call service /get_planning_scene");
    return;
  }

  AllowedCollisionMatrix currentACM = getCollisionMatrix(scene_srv, false);
  currentACM.entry_names.push_back(GRASPABLE_OBJECT);
  AllowedCollisionEntry entry;
  entry.enabled.resize(currentACM.entry_names.size());

  // Add new row to allowed collision matrix
  //TODO: Make objects allowed to collision only with gripper
  for (int i = 0; i < entry.enabled.size(); i++)
    entry.enabled[i] = true;

  currentACM.entry_values.push_back(entry);

  // Extend the last column of the matrix
  for (int i = 0; i < currentACM.entry_values.size(); i++)
    currentACM.entry_values[i].enabled.push_back(true);

  PlanningScene newSceneDiff;
  newSceneDiff.is_diff = true;
  newSceneDiff.allowed_collision_matrix = currentACM;
  planning_scene_diff_pub.publish(newSceneDiff);

  // To check that the matrix was properly updated
  if (!client_get_scene.call(scene_srv))
    ROS_WARN("Failed to call service /get_planning_scene");
  else
    getCollisionMatrix(scene_srv, true);
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
*/

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
  display_pub.publish(display_trajectory);
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
  // Define a goal pose
  group_.setPoseTarget(newPose(sample));
  group_.setPlanningTime(5.0); // 5s

  // Plan trayectory
  moveit::planning_interface::MoveGroup::Plan pose_plan;
  bool success = group_.plan(pose_plan);
  ROS_INFO("Visualizando plan 1 (Pose Goal) %s", success ? "" : "FAILED");
  // Sleep to give Rviz time to visualize the plan.
  //sleep(5.0);
  //if (success) visualizePlan(pose_plan);
  return success;
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
