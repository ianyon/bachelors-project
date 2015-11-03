//
// Created by yoko on 03-11-15.
//

#include "grasp_ranker.h"

#include <geometry_msgs/Point.h>

#include "utils.h"

namespace bachelors_final_project
{

detection::GraspRanker::GraspRanker(tf::TransformListener &tf_listener) :
    tf_listener_(tf_listener)
{ }

void detection::GraspRanker::configure(BoundingBoxPtr &obj_bounding_box_, std::string &end_effector)
{
  obj_ = obj_bounding_box_;
  end_effector_ = end_effector;
}

void detection::GraspRanker::rankGraspingPoses(const std::vector<moveit_msgs::Grasp> &side_grasps,
                                               const std::vector<moveit_msgs::Grasp> &top_grasps)
{
  rankSideGrasps(side_grasps);
}

void detection::GraspRanker::rankSideGrasps(const std::vector<moveit_msgs::Grasp> side_grasps)
{
  for (int i = 0; i < side_grasps.size(); ++i)
  {
    Eigen::Vector3f grasp_distance;
    /*transformPoint(end_effector_, obj_->OBJ_FRAME, Eigen::Vector3f(WRIST_TO_TOOL_DISTANCE, 0.0, 0.0), grasp_distance,
                   obj_->stamp_,
                   tf_listener_);*/

    const geometry_msgs::Point &position = side_grasps[i].grasp_pose.pose.position;
    tf::Quaternion orientation_tf;
    tf::quaternionMsgToTF(side_grasps[i].grasp_pose.pose.orientation, orientation_tf);
    Eigen::Quaternionf orientation(orientation_tf.w(), orientation_tf.x(), orientation_tf.y(), orientation_tf.z());
    Eigen::Vector3f position_obj_frame(position.x, position.y, position.z);
    Eigen::Vector3f real_grasp_position =
        orientation * Eigen::Vector3f(WRIST_TO_TOOL_DISTANCE, 0.0, 0.0) + position_obj_frame;

  }
  /*
   * Distance to object center. We favor grasps with a smaller distance to the object center.

• Grasp width. We reward grasp widths closer to a preferred width (0.08 m).

• Grasp orientation. Preference is given to grasps with a smaller angle between the line towards the shoulder and
the grasping direction.

• Distance from robot. We support grasps with a smaller distance to the shoulder.
   */
}

} // namespace bachelors_final_project