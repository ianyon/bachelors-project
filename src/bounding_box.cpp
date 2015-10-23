//
// Created by ian on 10/22/15.
//

#include "bounding_box.h"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

namespace bachelors_final_project
{

detection::BoundingBox::BoundingBox()
{

}

void detection::BoundingBox::build(CloudPtr &world_coords_planar_obj, CloudPtr &world_coords_obj, CloudPtr &obj)
{
  // Compute object centroid
  pcl::compute3DCentroid(*world_coords_planar_obj, world_coords_centroid_);
  Eigen::Matrix3f sensor_covariance;
  computeCovarianceMatrixNormalized(*world_coords_planar_obj, world_coords_centroid_, sensor_covariance);

  // Compute eigen vectors (principal directions)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(sensor_covariance);
  eigen_vectors_ = eigen_solver.eigenvectors();
  // Third component is orthogonal to principal axes
  eigen_vectors_.col(2) = eigen_vectors_.col(0).cross(eigen_vectors_.col(1));

  // Move the points of the object to it's own coordinates centered in the centroid
  Eigen::Affine3f transform = getWorldToObjectCentroidTransform();
  pcl::transformPointCloud(*world_coords_planar_obj, *obj, transform);

  getMinMax3D(*obj, min_point_centroid_, max_point_centroid_);
  middle_point_ = 0.5f * (max_point_centroid_.getVector3fMap() + min_point_centroid_.getVector3fMap());

  // Correct coordinates centering the object in the center of the bounding box
  transform = Eigen::Affine3f::Identity();
  pcl::transformPointCloud(*obj, *obj, transform.translate(-middle_point_));

  // Final transform: back to world coordinates
  obj_to_world_rotation_ = Eigen::Quaternionf(eigen_vectors_);
  obj_to_world_translation_ = eigen_vectors_ * middle_point_ + world_coords_centroid_.head<3>();

  computeHeight(world_coords_obj);
}

/**
 * Change coordinates from world's to object's
 */
Eigen::Affine3f detection::BoundingBox::getWorldToObjectCentroidTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Rotate to objects coordinates
  Eigen::Matrix3f world_to_obj_rotation(eigen_vectors_.transpose());
  transform.rotate(world_to_obj_rotation);

  // Translate to object's coordinates: Make the centroid be the origin
  // Rotate the centroid so it points to the rigth direction
  transform.translate(-1.f * (world_to_obj_rotation * world_coords_centroid_.head<3>()));
  return transform;
}

void detection::BoundingBox::computeHeight(CloudPtr &world_coords_obj)
{
  Eigen::Affine3f transform = getWorldToObjectCentroidTransform();
  CloudPtr obj3D;
  pcl::transformPointCloud(*world_coords_obj, *obj3D, transform);
  Point min_3d_point, max_3d_point;
  getMinMax3D(*world_coords_obj, min_3d_point, max_3d_point);
  heigth_3D_ = (max_3d_point.x - min_3d_point.x) / 2.0;
}
} // namespace bachelors_final_project