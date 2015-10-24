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
  planar_obj.reset(new Cloud);
}

void detection::BoundingBox::build(CloudPtr &world_coords_planar_obj, CloudPtr &world_coords_obj)
{
  // Compute object centroid
  pcl::compute3DCentroid(*world_coords_planar_obj, world_coords_planar_centroid_);
  Eigen::Matrix3f sensor_covariance;
  computeCovarianceMatrixNormalized(*world_coords_planar_obj, world_coords_planar_centroid_, sensor_covariance);

  // Compute eigen vectors (principal directions)
  //Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(sensor_covariance);
  eigen_solver.compute(sensor_covariance);
  eigen_vectors_ = eigen_solver.eigenvectors();
  // Third component is orthogonal to principal axes
  eigen_vectors_.col(2) = eigen_vectors_.col(0).cross(eigen_vectors_.col(1));

  // Move the points of the object to it's own coordinates centered in the centroid
  pcl::transformPointCloud(*world_coords_planar_obj, *planar_obj, getWorldToObjectCentroidTransform());

  getMinMax3D(*planar_obj, min_pt_planar_centroid_, max_pt_planar_centroid_);
  planar_shift_ = 0.5f * (max_pt_planar_centroid_.getVector3fMap() + min_pt_planar_centroid_.getVector3fMap());

  // Correct coordinates centering the object in the center of the bounding box
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  pcl::transformPointCloud(*planar_obj, *planar_obj, transform.translate(-planar_shift_));

  // Final transform: back to world coordinates
  obj_to_world_rotation_ = Eigen::Quaternionf(eigen_vectors_);
  obj_to_world_translation_ = eigen_vectors_ * planar_shift_ + world_coords_planar_centroid_.head<3>();

  computeHeight(world_coords_obj);

  world_coords_3D_pose = obj_to_world_translation_ + eigen_vectors_ * Eigen::Vector3f(heigth_3D_ / 2, 0, 0);
}

Eigen::Vector3f detection::BoundingBox::worldCoordsBoundingBoxPose()
{
  return world_coords_3D_pose;
}

/**
 * Change coordinates from world's to object's centroid.
 * First translate the object and then rotate it. Transforms are applied like right multiplication
 * so Transform = Identity * Rotation * Translate => Vector(final) = Transform * Vector(inicial)
 */
Eigen::Affine3f detection::BoundingBox::getWorldToObjectCentroidTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // Rotate to objects coordinates
  transform.rotate(eigen_vectors_.transpose());
  // Translate to object's coordinates: Make the centroid be the origin
  return transform.translate(-world_coords_planar_centroid_.head<3>());
}

/**;
 * Change coordinates from world's to object's
 */
Eigen::Affine3f detection::BoundingBox::getWorldToObjectTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Translate to object's coordinates: Make the centroid be the origin
  transform.translate(-world_coords_planar_centroid_.head<3>());

  // Rotate to objects coordinates
  transform.rotate(eigen_vectors_.transpose());

  // Translate to object's coordinates: Make the centroid be the origin
  return transform.translate(-min_pt_planar_centroid_.getVector3fMap() + planar_shift_);
}

/**
 * Invert transformation
 */
Eigen::Affine3f detection::BoundingBox::getObjectToWorldTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  return transform.translate(world_coords_planar_centroid_.head<3>()).rotate(eigen_vectors_).translate(planar_shift_);
}

Eigen::Vector3f detection::BoundingBox::getCentroidToBoundingBoxMiddleVector()
{
  return -getBoundingBoxOriginToCentroid() + planar_shift_;
}

Eigen::Vector3f detection::BoundingBox::getBoundingBoxOriginToCentroid()
{
  return -min_pt_planar_centroid_.getVector3fMap();
}

void detection::BoundingBox::computeHeight(CloudPtr &world_coords_obj)
{
  CloudPtr obj3D(new Cloud);
  pcl::transformPointCloud(*world_coords_obj, *obj3D, getWorldToObjectCentroidTransform());
  Point min_3d_point, max_3d_point;
  getMinMax3D(*obj3D, min_3d_point, max_3d_point);
  heigth_3D_ = fabs(max_3d_point.x - min_3d_point.x);
}
} // namespace bachelors_final_project