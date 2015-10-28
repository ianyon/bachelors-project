//
// Created by ian on 10/22/15.
//

#include "bounding_box.h"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>

#include <tf/transform_broadcaster.h>

#include "utils.h"

using boost::str;
using boost::format;

namespace bachelors_final_project
{

detection::BoundingBox::BoundingBox(std::string obj_frame) :
    planar_obj(new Cloud),
    OBJ_FRAME(obj_frame)
{
}

void detection::BoundingBox::buildPlanar(CloudPtr &world_coords_planar_obj)
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

  create2DSize();
  createBoundingBoxCenteredMembers();
  createWorldCenteredMembers();

  // Final transform: back to world coordinates
  rotation_kinect_frame_ = Eigen::Quaternionf(eigen_vectors_);
  position_2D_kinect_frame_ = eigen_vectors_ * planar_shift_ + world_coords_planar_centroid_.head<3>();
}



geometry_msgs::Pose detection::BoundingBox::computePose3DRobotFrame(tf::TransformListener &tf_listener)
{
  // We want the origin of the reference frame
  geometry_msgs::PoseStamped pose_bounding_box_frame;
  pose_bounding_box_frame.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped pose_robot_frame;
  if (!transformPose(OBJ_FRAME, FOOTPRINT_FRAME, pose_bounding_box_frame, pose_robot_frame, planar_obj->header.stamp,
                     tf_listener))
    throw ComputeFailedException("Transform Failed");

  return pose_robot_frame.pose;
}

Point detection::BoundingBox::computePosition2DRobotFrame(tf::TransformListener &tf_listener)
{
  Point pose_robot_frame;
  if (!transformPoint(kinect_frame_, FOOTPRINT_FRAME, position_2D_kinect_frame_, pose_robot_frame, 0, tf_listener))
    throw ComputeFailedException("Without transform can't find horizontal plane coords");
  return pose_robot_frame;
}
Eigen::Vector3f detection::BoundingBox::getSizeWithExternHeight(float height)
{
  return Eigen::Vector3f(size_2D_[0], size_2D_[1], height);
}

void detection::BoundingBox::create2DSize()
{
  Eigen::Vector3f size;
  size = max_pt_planar_centroid_.getVector3fMap() + (-min_pt_planar_centroid_.getVector3fMap());
  size_2D_ = Eigen::Vector2f(size[2],size[1]);
}

void detection::BoundingBox::create3DSize(float heigth_3D)
{
  size_3D_[0] = size_2D_[0];
  size_3D_[1] = size_2D_[1];
  size_3D_[2] = heigth_3D;
}

void detection::BoundingBox::createWorldCenteredMembers()
{
  min_pt_planar_world_ = pcl::transformPoint(min_pt_planar_, getObjectToWorldTransform());
  max_pt_planar_world_ = pcl::transformPoint(max_pt_planar_, getObjectToWorldTransform());
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

/**
 * Invert transformation
 */
Eigen::Affine3f detection::BoundingBox::getObjectToWorldTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  return transform.translate(world_coords_planar_centroid_.head<3>()).rotate(eigen_vectors_).translate(planar_shift_);
}

void detection::BoundingBox::build3DAndPublishFrame(CloudPtr &world_coords_obj, tf::TransformBroadcaster broadcaster)
{
  CloudPtr obj3D(new Cloud);
  pcl::transformPointCloud(*world_coords_obj, *obj3D, getWorldToObjectCentroidTransform());
  Point min_3d_point, max_3d_point;
  getMinMax3D(*obj3D, min_3d_point, max_3d_point);
  create3DSize((float) fabs(max_3d_point.x - min_3d_point.x));
  position_3D_kinect_frame_ = position_2D_kinect_frame_ - eigen_vectors_ * Eigen::Vector3f(getHeigth() / 2, 0, 0);

  broadcastFrameUpdate(broadcaster, position_3D_kinect_frame_);
}

void detection::BoundingBox::broadcast2DFrameUpdate(tf::TransformBroadcaster broadcaster)
{
  broadcastFrameUpdate(broadcaster, position_2D_kinect_frame_);
}

void detection::BoundingBox::broadcastFrameUpdate(tf::TransformBroadcaster broadcaster, Eigen::Vector3f &position)
{
  tf::Transform transform;
  if (ros::ok())
  {
    transform.setOrigin(tf::Vector3(position[0], position[1], position[2]));

    Eigen::Quaternionf b;
    b = Eigen::AngleAxisf(-90.0 * M_PI / 180.0, Eigen::Vector3f(0.0, 1.0, 0.0));
    b = rotation_kinect_frame_*b;
    tf::Quaternion quaternion(b.x(), b.y(), b.z(), b.w());
    transform.setRotation(quaternion);
    ros::Time tf_time(planar_obj->header.stamp / 1000000.0);
    //broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), kinect_frame_, OBJ_FRAME));
    broadcaster.sendTransform(tf::StampedTransform(transform, tf_time, kinect_frame_, OBJ_FRAME));
  }
}

void things(pcl::visualization::PCLVisualizer &viz, bachelors_final_project::detection::BoundingBox *box)
{
  viz.addCube(Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf::Identity(),
              box->max_pt_planar_centroid_.x - box->min_pt_planar_centroid_.x,
              box->max_pt_planar_centroid_.y - box->min_pt_planar_centroid_.y,
              box->max_pt_planar_centroid_.z - box->min_pt_planar_centroid_.z,
              "id");
}

void detection::BoundingBox::visualizeData()
{
  pcl::PointCloud<pcl::PointXYZRGB> brief;
  //brief.push_back(newPointXYZRGB(newPoint(size_2D_), 255, 0, 0));
  //brief.push_back(newPointXYZRGB(newPoint(size_2D_robot_frame_), 0, 255, 0));
  brief.push_back(newPointXYZRGB(min_pt_planar_centroid_, 255, 0, 0));
  brief.push_back(newPointXYZRGB(max_pt_planar_centroid_, 255, 0, 0));
  brief.push_back(newPointXYZRGB(min_pt_planar_world_, 0, 0, 255));
  brief.push_back(newPointXYZRGB(max_pt_planar_world_, 0, 0, 255));
  brief.push_back(newPointXYZRGB(min_pt_planar_, 255, 0, 0));
  brief.push_back(newPointXYZRGB(max_pt_planar_, 255, 0, 0));
  //brief.push_back(newPointXYZRGB(pose_2d_kinect_frame_, 0, 0, 255));

  pcl::visualization::CloudViewer viewer("TEST BOUNDING BOX");
  viewer.runOnVisualizationThreadOnce(callable);
  viewer.runOnVisualizationThreadOnce(boost::bind(&things, _1, this));
  while (!viewer.wasStopped())
  {
    viewer.showCloud(brief.makeShared());
    ros::WallDuration(2.0).sleep();
    viewer.showCloud(planar_obj);
    ros::WallDuration(2.0).sleep();
  }
}
} // namespace bachelors_final_project