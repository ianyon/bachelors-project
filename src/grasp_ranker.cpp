//
// Created by yoko on 03-11-15.
//

#include "grasp_ranker.h"

#include <boost/range/counting_range.hpp>

#include <geometry_msgs/Point.h>

#include "utils.h"


/**
 * Sort from minor to larger distance to the center of the object
 */
bool indirectCenterDistanceComparator(std::vector<Eigen::Vector3f> array, size_t first,
                                      size_t second)
{
  Eigen::Vector3f center;
  return (((array[first] - center).norm()) <= ((array[second] - center).norm()));
}

/**
 * Sort from minor to larger distance to the shoulder
 */
bool indirectShoulderDistanceComparator(Eigen::Vector3f shoulder_position, std::vector<Eigen::Vector3f> &array,
                                        size_t first,
                                        size_t second)
{ return (((array[first] - shoulder_position).norm()) <= ((array[second] - shoulder_position).norm())); }

/**
 * Sort from minor to larger angle difference
 */
bool indirectGraspOrientationComparator(Eigen::Vector4f shoulder_orientation, std::vector<Eigen::Vector3f> &array,
                                        size_t first,
                                        size_t second)
{
  Eigen::Vector4f first_grasping_direction = array[first].colwise().homogeneous();
  Eigen::Vector4f second_grasping_direction = array[second].colwise().homogeneous();

  return (pcl::getAngle3D(shoulder_orientation, first_grasping_direction) <=
          pcl::getAngle3D(shoulder_orientation, second_grasping_direction));
}

namespace bachelors_final_project
{

detection::GraspRanker::GraspRanker(tf::TransformListener &tf_listener) :
    tf_listener_(tf_listener)
{ }

void detection::GraspRanker::configure(BoundingBoxPtr &obj_bounding_box_, std::string end_effector,
                                       std::string shoulder_link)
{
  obj_ = obj_bounding_box_;
  end_effector_ = end_effector;
  shoulder_link_ = shoulder_link;
}

void detection::GraspRanker::rankGraspingPoses(const std::vector<moveit_msgs::Grasp> &side_grasps,
                                               const std::vector<moveit_msgs::Grasp> &top_grasps)
{
  rankGrasps(side_grasps,side_grasps_rank_);
}

/**
 * The score for a grasp is the sum of its positions in 4 lists ordered from minor to greater according to:
 *  - Distance to object center
 *  - Distance from robot
 *  - Grasp orientation
 *  - Grasp width
 */
void detection::GraspRanker::rankGrasps(const std::vector<moveit_msgs::Grasp> grasps,
                                        std::map<moveit_msgs::Grasp, int> &ranked_grasps)
{
  std::vector<Eigen::Vector3f> grasps_real_position;
  std::vector<size_t> grasps_index;

  for (int i = 0; i < grasps.size(); ++i)
  {
    const geometry_msgs::Point &position = grasps[i].grasp_pose.pose.position;
    tf::Quaternion orientation_tf;
    tf::quaternionMsgToTF(grasps[i].grasp_pose.pose.orientation, orientation_tf);
    Eigen::Quaternionf orientation(orientation_tf.w(), orientation_tf.x(), orientation_tf.y(), orientation_tf.z());
    Eigen::Vector3f position_obj_frame(position.x, position.y, position.z);
    Eigen::Vector3f real_grasp_position =
        orientation * Eigen::Vector3f(WRIST_TO_TOOL_DISTANCE, 0.0, 0.0) + position_obj_frame;
    grasps_real_position.push_back(real_grasp_position);
    grasps_index.push_back(i);
  }

  // Distance to object center: We favor grasps with a smaller distance to the object center.
  std::vector<size_t> distance_to_center_index = grasps_index;
  std::sort(distance_to_center_index.begin(), distance_to_center_index.end(),
            boost::bind(&indirectCenterDistanceComparator, &grasps_real_position, _1, _2));


  // Distance from robot: We support grasps with a smaller distance to the shoulder.
  std::vector<size_t> distance_to_shoulder_index = grasps_index;
  Eigen::Vector3f shoulder_position;
  transformPoint(shoulder_link_, obj_->OBJ_FRAME, Eigen::Vector3f(0, 0, 0), shoulder_position, obj_->stamp_,
                 tf_listener_);
  std::sort(distance_to_shoulder_index.begin(), distance_to_shoulder_index.end(),
            boost::bind(&indirectShoulderDistanceComparator, &shoulder_position, &grasps_real_position, _1, _2));


  // Grasp orientation: Prefer is grasps with a smaller angle between the line towards the shoulder and
  // the grasping direction
  std::vector<size_t> angle_difference_index = grasps_index;
  Eigen::Quaternionf obj_to_shoulder_orientation;
  Eigen::Vector4f shoulder_orientation = shoulder_position.colwise().homogeneous();
  std::sort(angle_difference_index.begin(), angle_difference_index.end(),
            boost::bind(&indirectGraspOrientationComparator, &shoulder_orientation, &grasps_real_position, _1, _2));

  // Grasp width: We reward grasp widths closer to a preferred width (0.08 m).


  // Set scores to 0
  BOOST_FOREACH(moveit_msgs::Grasp &grasp,grasps)
  {
    ranked_grasps[grasp] = 0;
  }

  // We use the position in the ordered index list for each cathegory as the score
  // The grasps scores are added in a random order because each list have a different order
  for (int j = 0; j < grasps_index.size(); ++j)
  {
    size_t grasp_index = distance_to_center_index[j];
    ranked_grasps[grasps[grasp_index]] += j;
    grasp_index = distance_to_shoulder_index[j];
    ranked_grasps[grasps[grasp_index]] += j;
    grasp_index = angle_difference_index[j];
    ranked_grasps[grasps[grasp_index]] += j;
  }
}

} // namespace bachelors_final_project