//
// Created by yoko on 03-11-15.
//

#ifndef PROJECT_GRASP_RANKER_H
#define PROJECT_GRASP_RANKER_H

#include <vector>

#include <moveit_msgs/Grasp.h>

#include "bounding_box.h"
#include "containers.h"

namespace bachelors_final_project
{
namespace detection
{


class GraspRanker
{
  tf::TransformListener &tf_listener_;
  std::string end_effector_, shoulder_link_;
  BoundingBoxPtr obj_;
  std::vector<int> side_ranking, top_ranking;

  RankedGrasps side_grasps_rank_, top_grasps_rank_;
  //std::map<moveit_msgs::Grasp, int> side_grasps_rank_, top_grasps_rank_;
  //typedef boost::bimap<boost::bimaps::set_of<moveit_msgs::Grasp,keyComp>, boost::bimaps::multiset_of<int> > RankedGrasps;
  //typedef RankedGrasps::left_value_type RankedGrasp;
  //RankedGrasps side_grasps_rank_, top_grasps_rank_;

  static bool rankedGraspComparator(RankedGrasp first, RankedGrasp second);

public:
  GraspRanker(tf::TransformListener &tf_listener);

  void configure(BoundingBoxPtr &obj_bounding_box_, std::string end_effector,
                 std::string shoulder_joint);

  void rankGraspingPoses(const std::vector<moveit_msgs::Grasp> &side_grasps,
                         const std::vector<moveit_msgs::Grasp> &top_grasps);

  void rankGrasps(const std::vector<moveit_msgs::Grasp> side_grasps,
                  RankedGrasps &ranked_grasps);

  void printRankedGrasps(RankedGrasps &ranked_grasps, std::string title);

  static bool indirectCenterDistanceComparator(std::vector<Vec3f> array, size_t first, size_t second);

  static bool indirectShoulderDistanceComparator(Vec3f shoulder_position, std::vector<Vec3f> &array, size_t first,
                                          size_t second);

  static bool indirectGraspOrientationComparator(Vec4f shoulder_orientation, std::vector<Vec3f> &array, size_t first,
                                          size_t second);


  void computeScoreAndRanking(RankedGrasps &ranked_grasps,
                              const std::vector<moveit_msgs::Grasp> &grasps,
                              std::vector<size_t> &distance_to_center_index,
                              std::vector<size_t> &distance_to_shoulder_index,
                              std::vector<size_t> &angle_difference_index);

  moveit_msgs::Grasp getSelectedGrasp();

  void normalize();
};

} // namespace detection
} // namespace bachelors_final_project

#endif //PROJECT_GRASP_RANKER_H
