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
  ros::Publisher collision_obj_pub, attached_obj_pub, actual_pose_pub_, pose_pub_;

  tf::TransformListener &tf_listener_;

  CloudPtr side_grasps_, top_grasps_;
  std::vector<moveit_msgs::Grasp> feasible_side_grasps_, feasible_top_grasps_;

  moveit::planning_interface::MoveGroup &r_arm_group_;

  std::string OBJ_FRAME;
  const std::string PLANNER_NAME_;

  geometry_msgs::Pose obj_pose_;

  /* How far from the grasp center should the wrist be. r_gripper_tool_frame Is the center of the finger tips.
   * ie: The center of the grasping point
   * rosrun tf tf_echo r_wrist_roll_link r_gripper_tool_frame
   *  At time 985.014
   *  - Translation: [0.180, 0.000, 0.000]
   *  - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
   *              in RPY [0.000, -0.000, 0.000]
   */
  double standoff;

  // Pregrasp distance 10 cm
  static const float PREGRASP_DISTANCE;

  bool generate_top_grasps_mayor_axis_, generate_top_grasps_minor_axis_;

  Vec3d side_grasp_center_, top_grasp_center_;

  PickupActionPtr &pick_action_client_;

  ros::NodeHandle nh_;

  moveit::planning_interface::MoveGroup gripper_group_;

  bool pick(moveit::planning_interface::MoveGroup &group, const std::string &object,
            moveit_msgs::Grasp &grasp);

public:
  GraspFilter(ros::NodeHandle &handle, tf::TransformListener &tf_listener,
              moveit::planning_interface::MoveGroup &r_arm_group,
              PickupActionPtr &pick_action_client, const std::string &planner_name);

  void configure(float side_grasp_height, float top_graps_height,
                 BoundingBoxPtr &obj_bounding_box, BoundingBoxPtr &table_bounding_box);

  void addSupportTable(Point &pose, Vec3f &size);

  void addCollisionObject(Vec3f &size, const std::string &frame);

  void filterGraspingPoses(CloudPtr side_grasps, CloudPtr top_grasps);


  bool processSample(Point &sample, bool generate_side_grasps, moveit_msgs::Grasp &grasp);

  trajectory_msgs::JointTrajectory generateGraspPosture(float value);

  void moveAllGripperJoints(float v);

  void openGripper();

  void closeGripper();

  moveit_msgs::Grasp generateTopGrasps(double x, double y, double z);

  moveit_msgs::Grasp generateSideGrasps(double x, double y, double z);

  moveit_msgs::Grasp tfTransformToGrasp(tf::Transform t);

  void publishGrasps(std::vector<moveit_msgs::Grasp> grasps);

  static bool generateSideSamples(BoundingBoxPtr &obj_bounding_box, float height);

  static bool generateTopSamples(BoundingBoxPtr &obj_bounding_box, float height,
                          bool &generate_mayor_axis, bool &generate_minor_axis);

  inline bool generateTopGraspsMinorAxis()
  { return generate_top_grasps_minor_axis_; }

  inline bool generateTopGraspsMayorAxis()
  { return generate_top_grasps_mayor_axis_; }

  inline const std::vector<moveit_msgs::Grasp> getSideGrasps()
  { return feasible_side_grasps_; }

  inline const std::vector<moveit_msgs::Grasp> getTopGrasps()
  { return feasible_top_grasps_; }

  inline void setParams(double standoff_)
  { standoff = standoff_; }

  inline std::string getEndEffectorLink()
  { return r_arm_group_.getEndEffectorLink(); }

  inline std::string getShoulderLink()
  { return "r_shoulder_pan_link"; }

  inline bool feasibleGrasps()
  { return getSideGrasps().size()!=0 || getTopGrasps().size()!=0; }
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_FILTER_H
