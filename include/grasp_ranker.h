//
// Created by yoko on 03-11-15.
//

#ifndef PROJECT_GRASP_RANKER_H
#define PROJECT_GRASP_RANKER_H

#include <vector>

#include <moveit_msgs/Grasp.h>

#include "bounding_box.h"

namespace bachelors_final_project
{
namespace detection
{
class GraspRanker
{
  tf::TransformListener &tf_listener_;
  std::string end_effector_;
  BoundingBoxPtr obj_;
  std::vector<int> side_ranking, top_ranking;

public:
  GraspRanker(tf::TransformListener &tf_listener);

  void configure(BoundingBoxPtr &obj_bounding_box_, std::string &end_effector);

  void rankGraspingPoses(const std::vector<moveit_msgs::Grasp> &side_grasps,
                         const std::vector<moveit_msgs::Grasp> &top_grasps);

  void rankSideGrasps(const std::vector<moveit_msgs::Grasp> side_grasps);
};

} // namespace detection
} // namespace bachelors_final_project

#endif //PROJECT_GRASP_RANKER_H
