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
  static const float SIDE_GRASP_HEIGHT_OBJ_BASE_FRAME_;

public:
  GraspSampler();

  void sampleSideGrasps(BoundingBoxPtr &);

  void sampleTopGrasps(BoundingBoxPtr &, bool generate_mayor_axis, bool generate_minor_axis);

  void sampleAxis(CloudPtr &, float, float, int, double, bool);

  inline const CloudPtr getSideGrasps() const
  { return side_grasps_; }

  inline const CloudPtr getTopGrasps() const
  { return top_grasps_; }

  /**
   * Grasp height returned is relative to the objects bounding box center.
   * Side grasps: The desired height is 2 cm above the table height
   * Top grasps: The desired height is the objects height */
  inline void computeGrapsHeight(double obj_height)
  {
    top_grasps_height_ = (float) (obj_height / 2);
    side_grasps_height_ = (float) (SIDE_GRASP_HEIGHT_OBJ_BASE_FRAME_ - (obj_height / 2));
  }

  inline float getSideGraspHeight()
  {
    return side_grasps_height_;
  }


  inline float getTopGraspHeight()
  {
    return top_grasps_height_;
  }

  void configure(BoundingBoxPtr &bounding_box);

  Cloud getSamples();

  bool generateSamples(BoundingBoxPtr &bounding_box, boost::function<bool(BoundingBoxPtr & , float)> generateSideGrasps,
                       boost::function<bool(BoundingBoxPtr & , float, bool & , bool & )> generateTopGrasps);

private:
  void numberOfSamples(const BoundingBoxPtr &bounding_box, int &minor_axis_samples, int &mayor_axis_samples);

  CloudPtr side_grasps_, top_grasps_;
  EllipseOperations ellipse_ops_;

  int top_grasp_samples;
};

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_GRASP_SAMPLER_H