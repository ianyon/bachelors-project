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
  void BoundingBox(PointT _min_pt, PointT _max_pt, Eigen::Quaternionf _rotation, Eigen::Vector3f _translation,
                    Eigen::Vector4f _centroid, Eigen::Matrix3f _eigen_vectors, Eigen::Vector3f _mean_diag,
                    double _heigth_3D):
    min_pt_(min_pt), max_pt_(max_pt), rotation_(rotation), translation_(translation), centroid_(centroid),
    eigen_vectors_(eigen_vectors), mean_diag_(mean_diag), heigth_3D_(heigth_3D);
  {
  }

  double getXLength()
  {
    return max_pt.x - min_pt.x;
  }

  double getYLength()
  {
    return max_pt.y - min_pt.y;
  }

  double getZLength()
  {
    return max_pt.z - min_pt.z;
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

typedef boost::shared_ptr<BoundingBox> BoundingBoxPtr;

struct RankedGrasps
{
  std::map<PointT, int> side_grasps, top_grasps;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_CONTAINERS_H
