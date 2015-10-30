#include "grasp_sampler.h"

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <utils.h>

#include "bounding_box.h"

using std::vector;

namespace bachelors_final_project
{
detection::GraspSampler::GraspSampler()
{
  side_grasps_.reset(new Cloud);
  top_grasps_.reset(new Cloud);

  top_grasp_samples = 50;
}

/**
 * Sample grasping poses in the object frame
 */
void detection::GraspSampler::sampleGraspingPoses(BoundingBoxPtr &bounding_box)
{
  side_grasps_->clear();
  sampleSideGrasps(bounding_box, side_grasps_);
  side_grasps_->header.frame_id = bounding_box->OBJ_FRAME;

  top_grasps_->clear();
  sampleTopGrasps(bounding_box, top_grasps_);
  top_grasps_->header.frame_id = bounding_box->OBJ_FRAME;
}

void detection::GraspSampler::sampleSideGrasps(BoundingBoxPtr &bounding_box, CloudPtr &side_grasps)
{
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
    if (!ellipse_ops_.getNewEllipsePoint(getSideGraspHeight(bounding_box->getHeight()), &ellipse_point))
      break;

    side_grasps->push_back(ellipse_point);
  }
}

void detection::GraspSampler::sampleTopGrasps(BoundingBoxPtr &bounding_box, CloudPtr &top_grasps)
{
  int minor_axis_samples, mayor_axis_samples;
  numberOfSamples(bounding_box, minor_axis_samples, mayor_axis_samples);

  // Grasping point at the center of the object's bounding box
  float height = (float) getTopGraspHeight(bounding_box->getHeight());

  pcl::PointXY bounding_box_min = bounding_box->getMin2D();
  double mayor_axis_step = bounding_box->getMayorAxisSize2D() / mayor_axis_samples;
  sampleAxis(top_grasps, height, bounding_box_min.x, mayor_axis_samples, mayor_axis_step, true);

  double minor_axis_step = bounding_box->getMinorAxisSize2D() / minor_axis_samples;
  sampleAxis(top_grasps, height, bounding_box_min.y, minor_axis_samples, minor_axis_step, false);
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

/**
 * Grasp height returned is relative to the objects bounding box center.
 * The desired height is 2 cm above the table height
 */
double detection::GraspSampler::getSideGraspHeight(double obj_height)
{
  return 0.02 - (obj_height / 2);
}

/**
 * Grasp height returned is relative to the objects bounding box center.
 * The desired height is the objects height
 */
double detection::GraspSampler::getTopGraspHeight(double obj_height)
{
  return obj_height / 2;
  //return -fmax(obj_height - 0.03, obj_height / 2);
}

}
