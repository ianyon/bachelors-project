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
  std::string end_effector_,shoulder_link_;
  BoundingBoxPtr obj_;
  std::vector<int> side_ranking, top_ranking;

  //std::vector<RankedGrasp> side_grasps_rank_, top_grasps_rank_;
  std::map<moveit_msgs::Grasp,int> side_grasps_rank_, top_grasps_rank_;

  //bool indirectCenterDistanceComparator(std::vector<Eigen::Vector3f> array, size_t first, size_t second);

public:
  GraspRanker(tf::TransformListener &tf_listener);

  void configure(BoundingBoxPtr &obj_bounding_box_, std::string end_effector,
                   std::string shoulder_joint);

  void rankGraspingPoses(const std::vector<moveit_msgs::Grasp> &side_grasps,
                         const std::vector<moveit_msgs::Grasp> &top_grasps);

  void rankGrasps(const std::vector<moveit_msgs::Grasp> side_grasps,
                  std::map<moveit_msgs::Grasp, int> &ranked_grasps);
};

} // namespace detection
} // namespace bachelors_final_project

#endif //PROJECT_GRASP_RANKER_H
