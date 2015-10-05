#include "grasp_sampler.h"
#include "ellipse_operations.h"

#include <pcl/common/transforms.h>

using std::vector;

namespace bachelors_final_project
{
detection::GraspSampler::GraspSampler()
{
  side_grasps_.reset(new PointCloudT);
  top_grasps_.reset(new PointCloudT);

  // 2 cm
  grasp_height_ = 0.02;
}

void detection::GraspSampler::sampleGraspingPoses(BoundingBox &bounding_box)
{
  side_grasps_->clear();
  sampleSideGrasps(bounding_box, side_grasps_);

  top_grasps_->clear();
  sampleTopGrasps(bounding_box, top_grasps_);
}

void detection::GraspSampler::sampleSideGrasps(BoundingBox &bounding_box, PointCloudTPtr &side_grasps)
{
  // We'll sample the points in the origin and then translate and rotate them
  double a = (bounding_box.max_pt.z - bounding_box.min_pt.z) / 2;
  double b = (bounding_box.max_pt.y - bounding_box.min_pt.y) / 2;
  double height = grasp_height_;

  // The transformation from 0,0 to the objects coordinate system
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translate(bounding_box.translation);
  transform.rotate(bounding_box.eigen_vectors);

  ellipse_ops_.setNumberOfPoints(50);
  ellipse_ops_.prepareComputation(a, b);

  PointT final_point, ellipse_point;
  // Continue while we can keep finding points
  while (ellipse_ops_.ellipsePointsLeft())
  {
    // Check if we found a next point
    if(!ellipse_ops_.getNewEllipsePoint(-height, &ellipse_point))
      break;

    // Now transform to the objects reference system
    final_point = pcl::transformPoint(ellipse_point, transform);

    side_grasps->push_back(ellipse_point);
    side_grasps->push_back(final_point);
  }
}


void detection::GraspSampler::sampleTopGrasps(BoundingBox &bounding_box, PointCloudTPtr &top_grasps)
{

}

}
