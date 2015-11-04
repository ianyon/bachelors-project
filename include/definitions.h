//
// Created by ian on 9/29/15.
//

#ifndef BACHELORS_FINAL_PROJECT_DEFINITIONS_H
#define BACHELORS_FINAL_PROJECT_DEFINITIONS_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit/move_group_interface/move_group.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace bachelors_final_project
{
static const std::string WORLD_FRAME = "/odom_combined";
static const std::string FOOTPRINT_FRAME = "/base_footprint";
static const std::string KINECT_TOPIC = "/head_mount_kinect/depth/points";
static const float TF_TIMEOUT = 2.0;
static const ros::Duration TF_DURATION(TF_TIMEOUT);

static const std::string TOOL_FRAME = "r_gripper_tool_frame";

/* Distance between r_wrist_roll_joint and r_gripper_tool_frame (Finger tip center)
 * rosrun tf tf_echo r_wrist_roll_link r_gripper_tool_frame
 *  At time 985.014
 *  - Translation: [0.180, 0.000, 0.000]
 *  - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
 *              in RPY [0.000, -0.000, 0.000]
 */
static const float WRIST_TO_TOOL_DISTANCE = 0.18;

enum FailedLevel { NORMALS, PLANE, OVER_TABLE, CLUSTERS };

typedef pcl::PointXYZ Point;
typedef pcl::Normal Normal;

typedef pcl::PointCloud <pcl::PointXYZ> Cloud;
typedef pcl::PointCloud <pcl::PointXYZ>::Ptr CloudPtr;
typedef pcl::PointCloud <pcl::PointXYZ>::ConstPtr CloudConstPtr;

typedef pcl::PointCloud <pcl::Normal> CloudNormal;
typedef pcl::PointCloud <pcl::Normal>::Ptr CloudNormalPtr;

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Vector3d Vec3d;

namespace detection
{
typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickupAction;
typedef boost::scoped_ptr<PickupAction> PickupActionPtr;
typedef boost::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
} // namespace detection

} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_DEFINITIONS_H
