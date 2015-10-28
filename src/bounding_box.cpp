//
// Created by ian on 10/22/15.
//

#include "bounding_box.h"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "tf2_ros/transform_broadcaster.h"
//#include <tf/transform_broadcaster.h>

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

  createSize();
  createBoundingBoxCenteredMembers();
  createWorldCenteredMembers();

  // Final transform: back to world coordinates
  rotation_kinect_frame_ = Eigen::Quaternionf(eigen_vectors_);
  position_2D_kinect_frame_ = eigen_vectors_ * planar_shift_ + world_coords_planar_centroid_.head<3>();
}

const Eigen::Vector3f &detection::BoundingBox::getSizePlanarFootprint()
{
  return size_planar_footprint_;
}

const Point detection::BoundingBox::getPlanarWorldPose()
{
  return pose_planar_world_;
}

Point detection::BoundingBox::computeFootprintPosePosition(tf2_ros::TransformListener &tf_listener)
{
  tf::Vector3 pose_kinect_frame(pose_planar_world_.x, pose_planar_world_.y, pose_planar_world_.z);

  Point footprint_pose;
  if (!transformPoint(kinect_frame_, FOOTPRINT_FRAME, pose_kinect_frame, footprint_pose, 0, tf_listener))
    throw ComputeFailedException("Without transform can't find horizontal plane coords");
  return footprint_pose;
}

geometry_msgs::Pose detection::BoundingBox::computeFootprintPose(tf2_ros::TransformListener &tf_listener)
{
  // We want the origin of the reference frame
  geometry_msgs::PoseStamped pose_bounding_box_frame;

  /*Eigen::Quaternionf quat(0.0, rotation_kinect_frame_.x(),
                          rotation_kinect_frame_.y(),
                          rotation_kinect_frame_.z());
  quat.normalize();

  pose_bounding_box_frame.pose.position.x = position_2D_kinect_frame_.x();
  pose_bounding_box_frame.pose.position.y = position_2D_kinect_frame_.y();
  pose_bounding_box_frame.pose.position.z = position_2D_kinect_frame_.z();

  pose_bounding_box_frame.pose.orientation.x = quat.x();
  pose_bounding_box_frame.pose.orientation.y = quat.y();
  pose_bounding_box_frame.pose.orientation.z = quat.z();
  pose_bounding_box_frame.pose.orientation.w = quat.w();*/

  geometry_msgs::PoseStamped pose_robot_frame;
  if (!transformPose(OBJ_FRAME, FOOTPRINT_FRAME, pose_bounding_box_frame, pose_robot_frame, planar_obj->header.stamp,
                     tf_listener))
    throw ComputeFailedException("Transform Failed");

  return pose_robot_frame.pose;
}

void detection::BoundingBox::createSize()
{
  size_planar_ = max_pt_planar_centroid_.getVector3fMap() + (-min_pt_planar_centroid_.getVector3fMap());
  size_planar_footprint_[0] = size_planar_[2];
  size_planar_footprint_[1] = size_planar_[1];
  size_planar_footprint_[2] = size_planar_[0]; // This should be 0 or 1 because we used a planar obj
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
  return position_3D_kinect_frame_;
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

void detection::BoundingBox::build3DAndPublishFrame(CloudPtr &world_coords_obj, tf2_ros::TransformBroadcaster broadcaster)
{
  CloudPtr obj3D(new Cloud);
  pcl::transformPointCloud(*world_coords_obj, *obj3D, getWorldToObjectCentroidTransform());
  Point min_3d_point, max_3d_point;
  getMinMax3D(*obj3D, min_3d_point, max_3d_point);
  heigth_3D_ = fabs(max_3d_point.x - min_3d_point.x);
  position_3D_kinect_frame_ = position_2D_kinect_frame_ + eigen_vectors_ * Eigen::Vector3f(heigth_3D_ / 2, 0, 0);

  broadcastFrameUpdate(broadcaster, position_3D_kinect_frame_);
}

void detection::BoundingBox::broadcast2DFrameUpdate(tf2_ros::TransformBroadcaster broadcaster)
{
  broadcastFrameUpdate(broadcaster, position_2D_kinect_frame_);
}

void detection::BoundingBox::broadcastFrameUpdate(tf2_ros::TransformBroadcaster broadcaster, Eigen::Vector3f &position)
{
  //tf::Transform transform;
  if (ros::ok())
  {
    geometry_msgs::TransformStamped transform;
    ros::Time tf_time(planar_obj->header.stamp / 1000000.0);
    transform.header.stamp = tf_time;//ros::Time::now();
    transform.header.frame_id = kinect_frame_;
    transform.child_frame_id = OBJ_FRAME;
    transform.transform.translation.x = position[0];
    transform.transform.translation.y = position[1];
    transform.transform.translation.z = position[2];
    //tf2::Quaternion q;
    //q.setRPY(0, 0, msg->theta);
    Eigen::Quaternionf q(eigen_vectors_);//.transpose());
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    broadcaster.sendTransform(transform);

    //transform.setOrigin(tf::Vector3(position[0], position[1], position[2]));
    //tf::Quaternion quaternion(q.x(), q.y(), q.z(), q.w());
    //Eigen::Quaterniond quat(eigen_quat.w(),eigen_quat.x(),eigen_quat.y(),eigen_quat.z());
    //tf::quaternionEigenToTF(quat, quaternion);
    //transform.setRotation(quaternion);
    //broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), kinect_frame_, OBJ_FRAME));
    //broadcaster.sendTransform(tf::StampedTransform(transform, tf_time, kinect_frame_, OBJ_FRAME));
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
  brief.push_back(newPointXYZRGB(newPoint(size_planar_), 255, 0, 0));
  brief.push_back(newPointXYZRGB(newPoint(size_planar_footprint_), 0, 255, 0));
  brief.push_back(newPointXYZRGB(min_pt_planar_centroid_, 255, 0, 0));
  brief.push_back(newPointXYZRGB(max_pt_planar_centroid_, 255, 0, 0));
  brief.push_back(newPointXYZRGB(min_pt_planar_world_, 0, 0, 255));
  brief.push_back(newPointXYZRGB(max_pt_planar_world_, 0, 0, 255));
  brief.push_back(newPointXYZRGB(min_pt_planar_, 255, 0, 0));
  brief.push_back(newPointXYZRGB(max_pt_planar_, 255, 0, 0));
  brief.push_back(newPointXYZRGB(pose_planar_world_, 0, 0, 255));

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