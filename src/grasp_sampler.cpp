#include "grasp_sampler.h"

#include <pcl/common/transforms.h>

using std::vector;

namespace bachelors_final_project
{
detection::GraspSampler::GraspSampler()
{
  // 2 cm
  grasp_heigth_ = 0.02;

  // pi/12[rad] = 15°
  ellipse_angular_step_ = (float) (M_PI / 12);

  size_t count = (size_t) floor(2 * M_PI / ellipse_angular_step_);
  theta_array_.resize((unsigned long) count);

  // Start clock-wise to ensure that the ones pointing at the robot
  // will have the exact step
  for (size_t i = 0; i < count; i++)
  {
    theta_array_[i] = -i * ellipse_angular_step_;
  }
}

void detection::GraspSampler::sampleGraspingPoses(BoundingBox &bounding_box)
{
  sampleSideGrasps(bounding_box, &side_grasps_);
  sampleTopGrasps(bounding_box, &top_grasps_);
}

void detection::GraspSampler::sampleSideGrasps(BoundingBox &bounding_box, vector<PointT> *side_grasps)
{
  // We'll sample the points in the origin and then translate and rotate them
  float h = 0; //bounding_box.mean_diag[0];
  float k = 0; //bounding_box.mean_diag[1];
  float a = bounding_box.eigen_vectors.col(0).norm();
  float b = bounding_box.eigen_vectors.col(1).norm();

  // Parametric ecuation of an ellipse with center in (h,k)
  // x = h + a*cos(alpha)
  // y = k + b*sin(alpha)
  // tg(theta) = b/a*tg(alpha)
  // x = h + cos(theta)/sqrt(cos(theta)^2/a^2 + sin(theta)^2/b^2)
  // y = k + sin(theta)/sqrt(cos(theta)^2/a^2 + sin(theta)^2/b^2)
  PointT final_point;
  for (size_t i = 0; i < theta_array_.size(); i++)
  {
    PointT ellipse_point;
    float theta = theta_array_[i];
    double square_root_term = sqrt(pow(cos(theta) / a, 2) + pow(sin(theta) / b, 2));
    ellipse_point.x = (float) (h + cos(theta) / square_root_term);
    ellipse_point.y = (float) (k + sin(theta) / square_root_term);
    ellipse_point.z = grasp_heigth_;

    // Now transform to the objects reference system
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(bounding_box.translation);
    transform.rotate(bounding_box.rotation);

    final_point = pcl::transformPoint(ellipse_point, transform);

    side_grasps->push_back(final_point);
  }
}

void detection::GraspSampler::sampleTopGrasps(BoundingBox &bounding_box, vector<PointT> *top_grasps)
{

}

}
