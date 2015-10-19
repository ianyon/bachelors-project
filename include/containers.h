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
namespace detection
{

class BoundingBox
{
public:
  BoundingBox()
  {
  }

  void initialize(Point min_pt, Point max_pt, Eigen::Quaternionf rotation, Eigen::Vector3f translation,
                  Eigen::Vector4f centroid, Eigen::Matrix3f eigen_vectors, Eigen::Vector3f mean_diag,
                  double heigth_3D)
  {
    min_pt_ = min_pt;
    max_pt_ = max_pt;
    rotation_ = rotation;
    translation_ = translation;
    centroid_ = centroid;
    rotation_ = rotation;
    eigen_vectors_ = eigen_vectors;
    mean_diag_ = mean_diag;
    heigth_3D_ = heigth_3D;
  }

  double getXLength()
  {
    return max_pt_.x - min_pt_.x;
  }

  double getYLength()
  {
    return max_pt_.y - min_pt_.y;
  }

  double getZLength()
  {
    return max_pt_.z - min_pt_.z;
  }

  Point min_pt_;
  Point max_pt_;
  // rotation_ represents the eigen vectors as a rotation_ matrix
  Eigen::Quaternionf rotation_;
  Eigen::Vector3f translation_;
  Eigen::Vector4f centroid_;
  Eigen::Matrix3f eigen_vectors_;
  Eigen::Vector3f mean_diag_;
  double heigth_3D_;
};

typedef boost::shared_ptr<BoundingBox> BoundingBoxPtr;

struct RankedGrasps
{
  std::map<Point, int> side_grasps, top_grasps;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_CONTAINERS_H
