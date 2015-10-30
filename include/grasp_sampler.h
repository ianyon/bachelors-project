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

  void sampleGraspingPoses(BoundingBoxPtr &bounding_box);

  void sampleSideGrasps(BoundingBoxPtr &, CloudPtr &);

  void sampleTopGrasps(BoundingBoxPtr &, CloudPtr &);

  void sampleAxis(CloudPtr &, float, float, int, double, bool);

  double getSideGraspHeight(double obj_height);
  double getTopGraspHeight(double obj_height);

  const CloudPtr getSideGrasps() const
  {
    return side_grasps_;
  }

  const CloudPtr getTopGrasps() const
  {
    return top_grasps_;
  }

private:
  void numberOfSamples(const BoundingBoxPtr &bounding_box, int &minor_axis_samples, int &mayor_axis_samples);

  CloudPtr side_grasps_, top_grasps_;
  EllipseOperations ellipse_ops_;

  int top_grasp_samples;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H