//
// Created by ian on 10/3/15.
//

#ifndef BACHELORS_FINAL_PROJECT_CONTAINERS_H
#define BACHELORS_FINAL_PROJECT_CONTAINERS_H

#include <vector>
#include <map>

#include "definitions.h"

namespace bachelors_final_project
{
namespace detection
{

struct BoundingBox
{
  void initialize(PointT _min_pt, PointT _max_pt, Eigen::Quaternionf _rotation, Eigen::Vector3f _translation,
                    Eigen::Vector4f _centroid, Eigen::Matrix3f _eigen_vectors, Eigen::Vector3f _mean_diag,
                    double _heigth_3D)
  {
    min_pt = _min_pt;
    max_pt = _max_pt;
    rotation = _rotation;
    translation = _translation;
    centroid = _centroid;
    eigen_vectors = _eigen_vectors;
    mean_diag = _mean_diag;
    heigth_3D = _heigth_3D;
  }

  PointT min_pt;
  PointT max_pt;
  // rotation represents the eigen vectors as a rotation matrix
  Eigen::Quaternionf rotation;
  Eigen::Vector3f translation;
  Eigen::Vector4f centroid;
  Eigen::Matrix3f eigen_vectors;
  Eigen::Vector3f mean_diag;
  double heigth_3D;
};

struct RankedGrasps
{
  std::map<PointT, int> side_grasps, top_grasps;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_CONTAINERS_H
