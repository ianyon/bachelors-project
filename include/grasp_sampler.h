#ifndef BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H
#define BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H

#include <vector>

#include "definitions.h"
#include "containers.h"
#include "ellipse_operations.h"

namespace bachelors_final_project
{
namespace detection
{

class GraspSampler
{
public:
  GraspSampler();

  void sampleGraspingPoses(BoundingBoxPtr &);

  void sampleSideGrasps(BoundingBoxPtr &, PointCloudPtr &);

  void sampleTopGrasps(BoundingBoxPtr &, PointCloudPtr &);

  void sampleAxis(PointCloudPtr &, Eigen::Affine3f &, float, float, float, int, double, bool);

  Eigen::Affine3f getTransform(const BoundingBoxPtr &bounding_box, bool side_grasp_transform) const;

  const PointCloudPtr getSideGrasps() const
  {
    return side_grasps_;
  }

  const PointCloudPtr getTopGrasps() const
  {
    return top_grasps_;
  }

private:
  PointCloudPtr side_grasps_, top_grasps_;
  double side_grasp_height_;
  EllipseOperations ellipse_ops_;
  int top_grasp_samples;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H