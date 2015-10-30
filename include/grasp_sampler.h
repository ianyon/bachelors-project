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
  float side_grasps_height_, top_grasps_height_;

public:
  GraspSampler();

  void sampleGraspingPoses(BoundingBoxPtr &bounding_box);

  void sampleSideGrasps(BoundingBoxPtr &, CloudPtr &);

  void sampleTopGrasps(BoundingBoxPtr &, CloudPtr &);

  void sampleAxis(CloudPtr &, float, float, int, double, bool);

  void computeGraspsHeight(double obj_height);

  inline const CloudPtr getSideGrasps() const
  { return side_grasps_; }

  inline const CloudPtr getTopGrasps() const
  { return top_grasps_; }

  inline float getTopGraspHeight()
  { return top_grasps_height_; } //return -fmax(obj_height - 0.03, obj_height / 2);

  inline float getSideGraspHeight()
  { return side_grasps_height_; }

private:
  void numberOfSamples(const BoundingBoxPtr &bounding_box, int &minor_axis_samples, int &mayor_axis_samples);

  CloudPtr side_grasps_, top_grasps_;
  EllipseOperations ellipse_ops_;

  int top_grasp_samples;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H