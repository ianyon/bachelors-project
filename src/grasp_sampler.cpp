#include "grasp_sampler.h"

#include <pcl/common/transforms.h>

using std::vector;

namespace bachelors_final_project
{
detection::GraspSampler::GraspSampler()
{
  side_grasps_.reset(new PointCloudT);
  top_grasps_.reset(new PointCloudT);

  // 2 cm
  grasp_heigth_ = 0.0;

  // pi/12[rad] = 15°
  ellipse_angular_step_ = (M_PI / 12)/4;

  size_t count = (size_t) floor(2 * M_PI / ellipse_angular_step_);
  theta_array_.resize((unsigned long) count);

  // Start clock-wise to ensure that the ones pointing at the robot
  // will have the exact step
  for (size_t i = 0; i < count; i++)
  {
    theta_array_[i] = i * ellipse_angular_step_;
  }
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
  double h = 0; //bounding_box.mean_diag[0];
  double k = 0; //bounding_box.mean_diag[1];
  double a = (bounding_box.max_pt.z - bounding_box.min_pt.z)/2;  //bounding_box.eigen_vectors.col(0).norm();
  double b = (bounding_box.max_pt.x - bounding_box.min_pt.x)/2;  //bounding_box.eigen_vectors.col(1).norm();

  // The transformation from 0,0 to the objects coordinate system
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translate(bounding_box.translation);
  //transform.rotate(bounding_box.rotation);
  transform.rotate(bounding_box.eigen_vectors);//.transpose());

  // Parametric ecuation of an ellipse with center in (h,k)
  // x = h + a*cos(alpha)
  // y = k + b*sin(alpha)
  // tg(theta) = b/a*tg(alpha)
  // x = h + cos(theta)/sqrt(cos(theta)^2/a^2 + sin(theta)^2/b^2)
  // y = k + sin(theta)/sqrt(cos(theta)^2/a^2 + sin(theta)^2/b^2)
  PointT final_point, ellipse_point;
  for (size_t i = 0; i < theta_array_.size(); i++)
  {
    double theta = theta_array_[i];
    double square_root_term = sqrt(pow(cos(theta) / a, 2) + pow(sin(theta) / b, 2));
    ellipse_point.x = (float) (k + sin(theta) / square_root_term);
    ellipse_point.y = (float) grasp_heigth_;
    ellipse_point.z = (float) (h + cos(theta) / square_root_term);

    // Now transform to the objects reference system
    final_point = pcl::transformPoint(ellipse_point, transform);

    side_grasps->push_back(final_point);
  }
}

void detection::GraspSampler::sampleTopGrasps(BoundingBox &bounding_box, PointCloudTPtr &top_grasps)
{

}

}
