//
// Created by ian on 10/6/15.
//

#ifndef BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
#define BACHELORS_FINAL_PROJECT_GRASP_FILTER_H

#include <string>
#include <ros/node_handle.h>

#include <actionlib/client/simple_action_client.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PickupAction.h>

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

  ros::Publisher display_pub, collision_obj_pub, attached_obj_pub, grasps_marker_, pose_pub_;

  ros::Publisher planning_scene_diff_pub;

  ros::ServiceClient client_get_scene;

  tf::TransformListener &tf_listener_;
  //tf::StampedTransform stamped_transform;

  CloudPtr side_grasps_, top_grasps_;
  CloudPtr feasible_side_grasps_, feasible_top_grasps_;

  typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;

  moveit::planning_interface::MoveGroup group_;

  //planning_scene::PlanningScene planning_scene;
  static const std::string GRASPABLE_OBJECT;
  static const std::string SUPPORT_TABLE;
  static const std::string WRIST_LINK;
  static const std::string GRIPPER_JOINT;
  static const std::string LBKPIECE_PLANNER;
  static const std::string RRTCONNECT_PLANNER;
  std::string OBJ_FRAME;
  const std::string PLANNER_NAME_;

  //std::string kinect_frame_id_;

  geometry_msgs::Pose obj_pose_;

  // How far from the grasp center should the wrist be: 3 cm
  double standoff;

  // Pregrasp distance 10 cm
  static const float PREGRASP_DISTANCE;

  bool generate_top_grasps_mayor_axis_, generate_top_grasps_minor_axis_;

  Eigen::Vector3d side_grasp_center_, top_grasp_center_;

  typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickupAction;
  typedef boost::scoped_ptr<PickupAction> PickupActionPtr;
  PickupActionPtr pick_action_client_;

  ros::NodeHandle nh_;

  void waitForAction(const PickupActionPtr &action, const ros::Duration &wait_for_server, const std::string &name);

  bool pick(moveit::planning_interface::MoveGroup &group, const std::string &object,
            const std::vector<moveit_msgs::Grasp> &grasps);

  void constructGoal(moveit::planning_interface::MoveGroup &group, moveit_msgs::PickupGoal &goal_out,
                     const std::string &object);

public:
  GraspFilter(ros::NodeHandle &handle, tf::TransformListener &tf_listener);

  void configure(float side_grasp_height, float top_graps_height,
                 BoundingBoxPtr &obj_bounding_box, BoundingBoxPtr &table_bounding_box);

  void addSupportTable(Point &pose, Eigen::Vector3f &size);

  void addCollisionObject(Eigen::Vector3f &size, const std::string &frame);

  void updateAllowedCollisionMatrix();

  void filterGraspingPoses(CloudPtr side_grasps, CloudPtr top_grasps);

  bool processSample(Point &sample, bool generate_side_grasps);

  std::vector<moveit_msgs::Grasp> generateTopGrasps(double x, double y, double z);

  std::vector<moveit_msgs::Grasp> generateSideGrasps(double x, double y, double z);

  moveit_msgs::Grasp tfTransformToGrasp(tf::Transform t);

  void publishGraspsAsMarkerarray(std::vector<moveit_msgs::Grasp> grasps);

  CloudPtr getSideGrasps();

  CloudPtr getTopGrasps();

  void setParams(double standoff);

  bool generateSideGrasps(BoundingBoxPtr &obj_bounding_box, float height);

  bool generateTopGrasps(BoundingBoxPtr &obj_bounding_box, float height);

  bool generateTopGraspsMinorAxis();

  bool generateTopGraspsMayorAxis();
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
