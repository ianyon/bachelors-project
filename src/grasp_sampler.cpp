#include "grasp_sampler.h"

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

using std::vector;

namespace bachelors_final_project
{
detection::GraspSampler::GraspSampler()
{
  side_grasps_.reset(new PointCloudT);
  top_grasps_.reset(new PointCloudT);

  // 2 cm
  side_grasp_height_ = 0.02;
  top_grasp_samples = 50;
}

void detection::GraspSampler::sampleGraspingPoses(BoundingBoxPtr &bounding_box)
{
  side_grasps_->clear();
  sampleSideGrasps(bounding_box, side_grasps_);

  top_grasps_->clear();
  sampleTopGrasps(bounding_box, top_grasps_);
}

void detection::GraspSampler::sampleSideGrasps(BoundingBoxPtr &bounding_box, PointCloudTPtr &side_grasps)
{
  // We'll sample the points in the origin and then translate and rotate them
  double a = (bounding_box->max_pt.z - bounding_box->min_pt.z) / 2;
  double b = (bounding_box->max_pt.y - bounding_box->min_pt.y) / 2;
  double height = side_grasp_height_;

  ellipse_ops_.setNumberOfPoints(50);
  ellipse_ops_.prepareComputation(a, b);

  PointT final_point, ellipse_point;
  // Continue while we can keep finding points
  while (ellipse_ops_.ellipsePointsLeft())
  {
    // Check if we found a next point
    if (!ellipse_ops_.getNewEllipsePoint(-height, &ellipse_point))
      break;

    // Now transform to the objects reference system
    final_point = pcl::transformPoint(ellipse_point, getTransform(bounding_box, true));

    //side_grasps->push_back(ellipse_point);
    side_grasps->push_back(final_point);
  }
}

void detection::GraspSampler::sampleTopGrasps(BoundingBoxPtr &bounding_box, PointCloudTPtr &top_grasps)
{
  Eigen::Affine3f transform = getTransform(bounding_box, false);

  // Compute the number of samples for each axis proportional to the relation of their lengths
  double mayor_axis_size = bounding_box->max_pt.z - bounding_box->min_pt.z;
  double minor_axis_size = bounding_box->max_pt.y - bounding_box->min_pt.y;
  int minor_axis_samples = (int) floor(top_grasp_samples * minor_axis_size / (minor_axis_size + mayor_axis_size));
  int mayor_axis_samples = (int) ceil(top_grasp_samples * mayor_axis_size / (minor_axis_size + mayor_axis_size));

  // Grasping point at the center of the object's bounding box
  float height = (float) (bounding_box->heigth_3D / 2.0);

  float minor_axis = bounding_box->mean_diag[1];
  float min_mayor_axis = bounding_box->min_pt.z;
  float max_mayor_axis = bounding_box->max_pt.z;
  double mayor_axis_step = (max_mayor_axis - min_mayor_axis) / mayor_axis_samples;
  sampleAxis(top_grasps, transform, minor_axis, height, min_mayor_axis, mayor_axis_samples, mayor_axis_step, true);

  float mayor_axis = bounding_box->mean_diag[2];
  float min_minor_axis = bounding_box->min_pt.y;
  float max_minor_axis = bounding_box->max_pt.y;
  double minor_axis_step = (max_minor_axis - min_minor_axis) / minor_axis_samples;
  sampleAxis(top_grasps, transform, mayor_axis, height, min_minor_axis, minor_axis_samples, minor_axis_step, false);
}

Eigen::Affine3f detection::GraspSampler::getTransform(const BoundingBoxPtr &bounding_box,
                                                      bool side_grasp_transform) const
{
  // The transformation from 0,0 to the objects coordinate system
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  if (side_grasp_transform)
    transform.translate(bounding_box->translation);
  else
    transform.translate(bounding_box->centroid.head<3>());
  transform.rotate(bounding_box->eigen_vectors);
  return transform;
}

void detection::GraspSampler::sampleAxis(PointCloudTPtr &top_grasps, Eigen::Affine3f &transform, float fixed_axis,
                                         float height, float min_axis, int n_samples, double step, bool is_mayor)
{
  PointT final_point, axis_sample;
  for (int i = 0; i < n_samples; ++i)
  {
    axis_sample.x = -height;

    if (is_mayor)
    {
      axis_sample.y = fixed_axis;
      axis_sample.z = (float) (i * step + min_axis);
    }
    else
    {
      axis_sample.y = (float) (i * step + min_axis);
      axis_sample.z = fixed_axis;
    }

    // Now transform to the objects reference system
    final_point = pcl::transformPoint(axis_sample, transform);
    top_grasps->push_back(final_point);
    //top_grasps->push_back(axis_sample);
  }
}

}
