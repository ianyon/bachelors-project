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

  void sampleGraspingPoses(BoundingBox &);

  void sampleSideGrasps(BoundingBox &, PointCloudTPtr &);

  void sampleTopGrasps(BoundingBox &, PointCloudTPtr &);

  const PointCloudTPtr getSideGrasps() const
  {
    return side_grasps_;
  }

  const PointCloudTPtr getTopGrasps() const
  {
    return top_grasps_;
  }

private:
  PointCloudTPtr side_grasps_, top_grasps_;
  double grasp_height_;
  EllipseOperations ellipse_ops_;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H