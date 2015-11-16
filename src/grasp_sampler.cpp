#include "grasp_sampler.h"

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include "utils.h"
#include "grasp_point_detector.h"

using std::vector;

namespace bachelors_final_project
{
// 0.05 is possible but there is high probability of r_forearm_link collision with table due to controller precision
const float detection::GraspSampler::SIDE_GRASP_HEIGHT_OBJ_BASE_FRAME_ = 0.06;

detection::GraspSampler::GraspSampler()
{
  side_grasps_.reset(new Cloud);
  top_grasps_.reset(new Cloud);

  top_grasp_samples = 50;
}

void detection::GraspSampler::configure(BoundingBoxPtr &bounding_box)
{
  computeGrapsHeight(bounding_box->getHeight());

  side_grasps_->clear();
  side_grasps_->header.frame_id = bounding_box->OBJ_FRAME;
  side_grasps_->header.stamp = bounding_box->stamp_;
  top_grasps_->clear();
  top_grasps_->header.frame_id = bounding_box->OBJ_FRAME;
  top_grasps_->header.stamp = bounding_box->stamp_;
}

Cloud detection::GraspSampler::getSamples()
{
  CloudPtr side_samples = getSideGrasps();
  CloudPtr top_samples = getTopGrasps();
  Cloud samples = *side_samples + *top_samples;
  return samples;
}

bool detection::GraspSampler::generateSamples(BoundingBoxPtr &bounding_box,
                                              boost::function<bool(BoundingBoxPtr &, float)> doSideSampling,
                                              boost::function<bool(BoundingBoxPtr &, float, bool &,
                                                                   bool &)> doTopSampling)
{
  if (doSideSampling(bounding_box, getSideGraspHeight()))
    sampleSideGrasps(bounding_box);

  bool generate_mayor_axis, generate_minor_axis;
  if (doTopSampling(bounding_box, getTopGraspHeight(), generate_mayor_axis, generate_minor_axis))
    sampleTopGrasps(bounding_box, generate_mayor_axis, generate_minor_axis);

  if (getSideGrasps()->size() == 0 && getTopGrasps()->size() == 0)
  {
    ROS_ERROR("No grasp samples. The object can't be grasped");
    return false;
  }
  return true;
}

void detection::GraspSampler::sampleSideGrasps(BoundingBoxPtr &bounding_box)
{
  ROS_INFO_NAMED(DETECTION(), "Sampling side grasps.");
  // We'll sample the points in the origin and then translate and rotate them
  double a = bounding_box->getMayorAxisSize2D() / 2;
  double b = bounding_box->getMinorAxisSize2D() / 2;

  ellipse_ops_.setNumberOfPoints(50);
  ellipse_ops_.prepareComputation(a, b);

  Point ellipse_point;
  // Continue while we can keep finding points
  while (ellipse_ops_.ellipsePointsLeft())
  {
    // Check if we found a next point
    if (!ellipse_ops_.getNewEllipsePoint(getSideGraspHeight(), &ellipse_point))
      break;

    side_grasps_->push_back(ellipse_point);
  }
}

void detection::GraspSampler::sampleTopGrasps(BoundingBoxPtr &bounding_box, bool generate_mayor_axis,
                                              bool generate_minor_axis)
{
  ROS_INFO_NAMED(DETECTION(), "Sampling top grasps: %s %s axis", generate_mayor_axis ? "mayor" : "",
                 generate_minor_axis ? "and minor" : "");
  int minor_axis_samples, mayor_axis_samples;
  numberOfSamples(bounding_box, minor_axis_samples, mayor_axis_samples);

  // Grasping point at the center of the object's bounding box
  float height = getTopGraspHeight();

  pcl::PointXY bounding_box_min = bounding_box->getMin2D();
  double mayor_axis_step = bounding_box->getMayorAxisSize2D() / mayor_axis_samples;
  if (generate_mayor_axis)
    sampleAxis(top_grasps_, height, bounding_box_min.x, mayor_axis_samples, mayor_axis_step, true);

  double minor_axis_step = bounding_box->getMinorAxisSize2D() / minor_axis_samples;
  if (generate_minor_axis)
    sampleAxis(top_grasps_, height, bounding_box_min.y, minor_axis_samples, minor_axis_step, false);
}

/**
 * Compute the number of samples for each axis proportional to the relation of their lengths
 */
void detection::GraspSampler::numberOfSamples(const BoundingBoxPtr &bounding_box, int &minor_axis_samples,
                                              int &mayor_axis_samples)
{
  double mayor_axis_size = bounding_box->getMayorAxisSize2D();
  double minor_axis_size = bounding_box->getMinorAxisSize2D();
  const double both_axis_size = minor_axis_size + mayor_axis_size;
  minor_axis_samples = (int) floor(top_grasp_samples * minor_axis_size / both_axis_size);
  mayor_axis_samples = (int) ceil(top_grasp_samples * mayor_axis_size / both_axis_size);
}

void detection::GraspSampler::sampleAxis(CloudPtr &top_grasps, float height, float min_axis, int n_samples, double step,
                                         bool is_mayor_axis)
{
  for (int i = 0; i < n_samples; ++i)
  {
    if (is_mayor_axis)
      top_grasps->push_back(Point((float) (i * step + min_axis), 0, height));
    else
      top_grasps->push_back(Point(0, (float) (i * step + min_axis), height));
  }
}

} // namespace bachelors_final_project
