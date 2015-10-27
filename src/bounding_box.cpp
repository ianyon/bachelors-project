//
// Created by ian on 10/22/15.
//

#include "bounding_box.h"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>
#include <utils.h>

namespace bachelors_final_project
{

detection::BoundingBox::BoundingBox() :
    planar_obj(new Cloud)
{
}

void detection::BoundingBox::build(CloudPtr &world_coords_planar_obj)
{
  kinect_frame_ = world_coords_planar_obj->header.frame_id;

  // Compute object centroid
  pcl::compute3DCentroid(*world_coords_planar_obj, world_coords_planar_centroid_);
  Eigen::Matrix3f sensor_covariance;
  computeCovarianceMatrixNormalized(*world_coords_planar_obj, world_coords_planar_centroid_, sensor_covariance);

  // Compute eigen vectors (principal directions)
  eigen_solver.compute(sensor_covariance);
  eigen_vectors_ = eigen_solver.eigenvectors();
  // Third component is orthogonal to principal axes
  eigen_vectors_.col(2) = eigen_vectors_.col(0).cross(eigen_vectors_.col(1));

  // Move the points of the object to it's own coordinates centered in the centroid
  pcl::transformPointCloud(*world_coords_planar_obj, *planar_obj, getWorldToObjectCentroidTransform());

  getMinMax3D(*planar_obj, min_pt_planar_centroid_, max_pt_planar_centroid_);
  planar_shift_ = 0.5f * (max_pt_planar_centroid_.getVector3fMap() + min_pt_planar_centroid_.getVector3fMap());

  createSize();
  createBoundingBoxCenteredMembers();
  createWorldCenteredMembers();

  // Final transform: back to world coordinates
  obj_to_world_rotation_ = Eigen::Quaternionf(eigen_vectors_);
  obj_to_world_translation_ = eigen_vectors_ * planar_shift_ + world_coords_planar_centroid_.head<3>();
}

const Eigen::Vector3f &detection::BoundingBox::getSizePlanarFootprint()
{
  return size_planar_footprint_;
}

const Point detection::BoundingBox::getPlanarWorldPose()
{
  return pose_planar_world_;
}

Point detection::BoundingBox::computeFootprintPose(tf::TransformListener &tf_listener)
{
  tf::Vector3 pose_kinect_frame(pose_planar_world_.x, pose_planar_world_.y, pose_planar_world_.z);

  Point footprint_pose;
  if (!transformPoint(kinect_frame_, FOOTPRINT_FRAME, pose_kinect_frame, footprint_pose, 0, tf_listener))
    throw ComputeFailedException("Without transform can't find horizontal plane coords");
  return footprint_pose;
}

void detection::BoundingBox::createSize()
{
  size_planar_ = max_pt_planar_centroid_.getVector3fMap() + (-min_pt_planar_centroid_.getVector3fMap());
  size_planar_footprint_[0] = size_planar_[2];
  size_planar_footprint_[1] = size_planar_[1];
  size_planar_footprint_[2] = size_planar_[0]; // This should be 0 or 1 because we used a planar obj
}

void things(pcl::visualization::PCLVisualizer &viz, bachelors_final_project::detection::BoundingBox *box)
{
  viz.addCube(Eigen::Vector3f(0,0,0), Eigen::Quaternionf::Identity(),
              box->max_pt_planar_centroid_.x - box->min_pt_planar_centroid_.x,
              box->max_pt_planar_centroid_.y - box->min_pt_planar_centroid_.y,
              box->max_pt_planar_centroid_.z - box->min_pt_planar_centroid_.z,
              "id");
}

void detection::BoundingBox::visualizeData()
{
  pcl::PointCloud<pcl::PointXYZRGB> brief;
  brief.push_back(newPointXYZRGB(newPoint(size_planar_),255,0,0));
  brief.push_back(newPointXYZRGB(newPoint(size_planar_footprint_),0,255,0));
  brief.push_back(newPointXYZRGB(min_pt_planar_centroid_,255,0,0));
  brief.push_back(newPointXYZRGB(max_pt_planar_centroid_,255,0,0));
  brief.push_back(newPointXYZRGB(min_pt_planar_world_,0,0,255));
  brief.push_back(newPointXYZRGB(max_pt_planar_world_,0,0,255));
  brief.push_back(newPointXYZRGB(min_pt_planar_,255,0,0));
  brief.push_back(newPointXYZRGB(max_pt_planar_,255,0,0));
  brief.push_back(newPointXYZRGB(pose_planar_world_,0,0,255));

  pcl::visualization::CloudViewer viewer("TEST BOUNDING BOX");
  viewer.runOnVisualizationThreadOnce(callable);
  viewer.runOnVisualizationThreadOnce(boost::bind(&things,_1,this));
  while(!viewer.wasStopped())
  {
    viewer.showCloud(brief.makeShared());
    ros::WallDuration(2.0).sleep();
    viewer.showCloud(planar_obj);
    ros::WallDuration(2.0).sleep();
  }
}

void detection::BoundingBox::createWorldCenteredMembers()
{
  min_pt_planar_world_ = pcl::transformPoint(min_pt_planar_, getObjectToWorldTransform());
  max_pt_planar_world_ = pcl::transformPoint(max_pt_planar_, getObjectToWorldTransform());
  pose_planar_world_ = pcl::transformPoint(Point(), getObjectToWorldTransform());
}

void detection::BoundingBox::createBoundingBoxCenteredMembers()
{
  // Correct coordinates centering the object in the center of the bounding box
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  pcl::transformPointCloud(*planar_obj, *planar_obj, translateCentroidToBoundingBox(t));
  min_pt_planar_ = pcl::transformPoint(min_pt_planar_centroid_, translateCentroidToBoundingBox(t));
  max_pt_planar_ = pcl::transformPoint(max_pt_planar_centroid_, translateCentroidToBoundingBox(t));
}

Eigen::Affine3f detection::BoundingBox::translateCentroidToBoundingBox(Eigen::Affine3f transform)
{
  return transform.translate(-planar_shift_);
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
  return translateCentroidToBoundingBox(getWorldToObjectCentroidTransform());
}

/**
 * Invert transformation
 */
Eigen::Affine3f detection::BoundingBox::getObjectToWorldTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  return transform.translate(world_coords_planar_centroid_.head<3>()).rotate(eigen_vectors_).translate(planar_shift_);
}

void detection::BoundingBox::computeHeight(CloudPtr &world_coords_obj)
{
  CloudPtr obj3D(new Cloud);
  pcl::transformPointCloud(*world_coords_obj, *obj3D, getWorldToObjectCentroidTransform());
  Point min_3d_point, max_3d_point;
  getMinMax3D(*obj3D, min_3d_point, max_3d_point);
  heigth_3D_ = fabs(max_3d_point.x - min_3d_point.x);
  world_coords_3D_pose = obj_to_world_translation_ + eigen_vectors_ * Eigen::Vector3f(heigth_3D_ / 2, 0, 0);
}
} // namespace bachelors_final_project