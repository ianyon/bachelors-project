//
// Created by ian on 10/3/15.
//

#ifndef BACHELORS_FINAL_PROJECT_CONTAINERS_H
#define BACHELORS_FINAL_PROJECT_CONTAINERS_H

#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>

#include "definitions.h"

namespace bachelors_final_project
{

struct VisualizerParams
{
  VisualizerParams(int normals_count_, float normals_size) :
      normals_count(normals_count_), normals_size(normals_size)
  { }

  int normals_count;
  float normals_size;
};

namespace detection
{

struct RankedGrasp
{
  RankedGrasp(moveit_msgs::Grasp _grasp, int _score) :
      grasp(_grasp), score(_score)
  { }

  moveit_msgs::Grasp grasp;
  int score;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_CONTAINERS_H
