#ifndef BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H
#define BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H

#include <vector>
#include <Eigen/Core>

#include "definitions.h"
#include "containers.h"

namespace bachelors_final_project
{
namespace detection
{

class GraspSampler
{
public:
  GraspSampler();

  void sampleGraspingPoses(BoundingBox &);

  void sampleSideGrasps(BoundingBox &, std::vector<PointT> *);

  void sampleTopGrasps(BoundingBox &, std::vector<PointT> *);

  const std::vector<PointT> &getSideGrasps() const
  {
    return side_grasps_;
  }

  const std::vector<PointT> &getTopGrasps() const
  {
    return top_grasps_;
  }

private:
  std::vector<PointT> side_grasps_, top_grasps_;
  float grasp_heigth_;
  float ellipse_angular_step_;
  std::vector<float> theta_array_;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H