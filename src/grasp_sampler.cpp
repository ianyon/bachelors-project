#include "grasp_sampler.h"

#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

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

void detection::GraspSampler::sampleGraspingPoses(BoundingBoxPtr &bounding_box, std::string kinect_frame)
{
  side_grasps_->clear();
  sampleSideGrasps(bounding_box, side_grasps_);
  side_grasps_->header.frame_id = kinect_frame;

  top_grasps_->clear();
  sampleTopGrasps(bounding_box, top_grasps_);
  top_grasps_->header.frame_id = kinect_frame;
}

void detection::GraspSampler::sampleSideGrasps(BoundingBoxPtr &bounding_box, CloudPtr &side_grasps)
{
  // We'll sample the points in the origin and then translate and rotate them
  double a = (bounding_box->max_pt_planar_centroid_.z - bounding_box->min_pt_planar_centroid_.z) / 2;
  double b = (bounding_box->max_pt_planar_centroid_.y - bounding_box->min_pt_planar_centroid_.y) / 2;

  ellipse_ops_.setNumberOfPoints(50);
  ellipse_ops_.prepareComputation(a, b);

  Point final_point, ellipse_point;
  // Continue while we can keep finding points
  while (ellipse_ops_.ellipsePointsLeft())
  {
    // Check if we found a next point
    if (!ellipse_ops_.getNewEllipsePoint(getSideGraspHeight(), &ellipse_point))
      break;

    // Now transform to the objects reference system
    final_point = pcl::transformPoint(ellipse_point, getTransform(bounding_box, true));

    //side_grasps->push_back(ellipse_point);
    side_grasps->push_back(final_point);
  }
}

void detection::GraspSampler::sampleTopGrasps(BoundingBoxPtr &bounding_box, CloudPtr &top_grasps)
{
  Eigen::Affine3f transform = getTransform(bounding_box, false);

  int minor_axis_samples, mayor_axis_samples;
  numberOfSamples(bounding_box, minor_axis_samples, mayor_axis_samples);

  // Grasping point at the center of the object's bounding box
  float height = (float) getTopGraspHeight(bounding_box->heigth_3D_);

  float minor_axis = bounding_box->planar_shift_[1];
  float min_mayor_axis = bounding_box->min_pt_planar_centroid_.z;
  float max_mayor_axis = bounding_box->max_pt_planar_centroid_.z;
  double mayor_axis_step = (max_mayor_axis - min_mayor_axis) / mayor_axis_samples;
  sampleAxis(top_grasps, transform, minor_axis, height, min_mayor_axis, mayor_axis_samples, mayor_axis_step, true);

  float mayor_axis = bounding_box->planar_shift_[2];
  float min_minor_axis = bounding_box->min_pt_planar_centroid_.y;
  float max_minor_axis = bounding_box->max_pt_planar_centroid_.y;
  double minor_axis_step = (max_minor_axis - min_minor_axis) / minor_axis_samples;
  sampleAxis(top_grasps, transform, mayor_axis, height, min_minor_axis, minor_axis_samples, minor_axis_step, false);
}

/**
 * Compute the number of samples for each axis proportional to the relation of their lengths
 */
void detection::GraspSampler::numberOfSamples(const BoundingBoxPtr &bounding_box, int &minor_axis_samples,
                                   int &mayor_axis_samples)
{
  double mayor_axis_size = bounding_box->max_pt_planar_centroid_.z - bounding_box->min_pt_planar_centroid_.z;
  double minor_axis_size = bounding_box->max_pt_planar_centroid_.y - bounding_box->min_pt_planar_centroid_.y;
  minor_axis_samples= (int) floor(top_grasp_samples * minor_axis_size / (minor_axis_size + mayor_axis_size));
  mayor_axis_samples= (int) ceil(top_grasp_samples * mayor_axis_size / (minor_axis_size + mayor_axis_size));
}

Eigen::Affine3f detection::GraspSampler::getTransform(const BoundingBoxPtr &bounding_box,
                                                      bool side_grasp_transform) const
{
  // The transformation from 0,0 to the objects coordinate system
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  if (side_grasp_transform)
    transform.translate(bounding_box->obj_to_world_translation_);
  else
    transform.translate(bounding_box->world_coords_planar_centroid_.head<3>());
  transform.rotate(bounding_box->eigen_vectors_);
  return transform;
}

void detection::GraspSampler::sampleAxis(CloudPtr &top_grasps, Eigen::Affine3f &transform, float fixed_axis,
                                         float height, float min_axis, int n_samples, double step, bool is_mayor)
{
  Point final_point, axis_sample;
  for (int i = 0; i < n_samples; ++i)
  {
    axis_sample.x = height;

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

double detection::GraspSampler::getSideGraspHeight()
{
    return -0.01;
}
double detection::GraspSampler::getTopGraspHeight(double obj_height)
{
    return -fmax(obj_height-0.03,obj_height/2);
}

}
