//
// Created by ian on 10/22/15.
//

#include "bounding_box.h"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>

#include <tf/LinearMath/Vector3.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "utils.h"

using boost::str;
using boost::format;

namespace bachelors_final_project
{

detection::BoundingBox::BoundingBox(std::string obj_frame) :
    planar_obj(new Cloud),
    obj_2D_kinect_frame(new Cloud),
    OBJ_FRAME(obj_frame)
{
}

void detection::BoundingBox::computeAndPublish(CloudPtr &obj_kinect_frame, pcl::ModelCoefficientsPtr table_plane_,
                                               tf::TransformBroadcaster tf_broadcaster)
{
  // Project to table to obtain object projection
  projectOnPlane(obj_kinect_frame, table_plane_, obj_2D_kinect_frame);
  // obj_2D_kinect_frame lies in plane YZ with a = blue (z) - b = green (y)

  buildPlanar(obj_2D_kinect_frame);

  build3DAndPublishFrame(obj_kinect_frame, tf_broadcaster);
}

void detection::BoundingBox::buildPlanar(CloudPtr &planar_obj_kinect_frame)
{
  kinect_frame_ = planar_obj_kinect_frame->header.frame_id;
  stamp_ = planar_obj_kinect_frame->header.stamp;

  // Compute object centroid
  pcl::compute3DCentroid(*planar_obj_kinect_frame, centroid_2D_kinect_frame_);
  Eigen::Matrix3f sensor_covariance;
  computeCovarianceMatrixNormalized(*planar_obj_kinect_frame, centroid_2D_kinect_frame_, sensor_covariance);

  // Compute eigen vectors (principal directions)
  eigen_solver.compute(sensor_covariance);
  eigen_vectors_ = eigen_solver.eigenvectors();
  // Third component is orthogonal to principal axes
  eigen_vectors_.col(2) = eigen_vectors_.col(0).cross(eigen_vectors_.col(1));

  // We check that all the eigen vectors point Z upwards
  if (eigen_vectors_(0, 0) < 0)
  {
    eigen_vectors_.col(0) *= -1.0;
    eigen_vectors_.col(2) *= -1.0;
  }

  // Move the points of the object to it's own coordinates centered in the centroid
  pcl::transformPointCloud(*planar_obj_kinect_frame, *planar_obj, getKinectToCentroidTransform());

  getMinMax3D(*planar_obj, min_pt_planar_centroid_, max_pt_planar_centroid_);
  planar_shift_ = 0.5f * (max_pt_planar_centroid_.getVector3fMap() + min_pt_planar_centroid_.getVector3fMap());

  createObjCenteredMembers();
  create2DSize();

  // Final transform: back to kinect coordinates
  rotation_kinect_frame_ = createRotationQuaternion2D(eigen_vectors_);
  position_base_kinect_frame_ = getCentroidToKinectTransform()*planar_shift_;
}

void detection::BoundingBox::build3DAndPublishFrame(CloudPtr &world_coords_obj, tf::TransformBroadcaster broadcaster)
{
  Cloud obj3D;
  pcl::transformPointCloud(*world_coords_obj, obj3D, Eigen::Affine3f(getInverseRotationQuaternion()));
  Cloud obj2D;
  pcl::transformPointCloud(*obj_2D_kinect_frame, obj2D, Eigen::Affine3f(getInverseRotationQuaternion()));
  Point min_3d_point, max_3d_point;
  // The size is the maximum heigth of the object minus the height of the table
  getMinMax3D(obj3D, min_3d_point, max_3d_point);
  float height_3D = max_3d_point.z - obj2D.points[0].z;
  // Set the size with height
  size_3D_ = Eigen::Vector3f(size_2D_[0], size_2D_[1], (float) fabs(height_3D));

  position_3D_kinect_frame_ =
      position_base_kinect_frame_ + rotation_kinect_frame_ * Eigen::Vector3f(0.0, 0.0, height_3D / 2);

  broadcastFrameUpdate(broadcaster, position_3D_kinect_frame_);
}

void detection::BoundingBox::broadcast2DFrameUpdate(tf::TransformBroadcaster broadcaster)
{
  broadcastFrameUpdate(broadcaster, position_base_kinect_frame_);
}

Eigen::Quaternionf detection::BoundingBox::createRotationQuaternion2D(Eigen::Matrix3f eigen_vectors)
{
  Eigen::Quaternionf eigen_q(eigen_vectors);
  tf::Quaternion quaternion(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());
  // Rotate the coordinate system to keep Z axis pointing upwards.
  quaternion *= tf::createQuaternionFromRPY(0.0, pcl::deg2rad(90.0), 0.0);
  return Eigen::Quaternionf(quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()).normalized();
}

void detection::BoundingBox::broadcastFrameUpdate(tf::TransformBroadcaster broadcaster, Eigen::Vector3f &position)
{
  tf::Transform transform;
  if (ros::ok())
  {
    transform.setOrigin(tf::Vector3(position[0], position[1], position[2]));
    transform.setRotation(getRotationQuaternionTF());
    ros::Time tf_time(stamp_ / 1000000.0);
    broadcaster.sendTransform(tf::StampedTransform(transform, tf_time, kinect_frame_, OBJ_FRAME));
  }
}

geometry_msgs::Pose detection::BoundingBox::computePose3DRobotFrame(tf::TransformListener &tf_listener)
{
  // We want the origin of the reference frame
  geometry_msgs::PoseStamped pose_bounding_box_frame;
  pose_bounding_box_frame.pose.orientation.w = 1.0;

  geometry_msgs::PoseStamped pose_robot_frame;
  if (!transformPose(OBJ_FRAME, FOOTPRINT_FRAME, pose_bounding_box_frame, pose_robot_frame, stamp_, tf_listener))
    throw ComputeFailedException("Transform Failed");

  return pose_robot_frame.pose;
}

Point detection::BoundingBox::computePosition2DRobotFrame(tf::TransformListener &tf_listener)
{
  Point pose_robot_frame;
  if (!transformPoint(kinect_frame_, FOOTPRINT_FRAME, position_base_kinect_frame_, pose_robot_frame, 0, tf_listener))
    throw ComputeFailedException("Without transform can't find horizontal plane coords");
  return pose_robot_frame;
}

void detection::BoundingBox::create2DSize()
{
  Eigen::Vector3f size;
  size = max_pt_planar_.getVector3fMap() + (-min_pt_planar_.getVector3fMap());
  // z is mayor axis so we make it the first component
  size_2D_ = Eigen::Vector2f(size[2], size[1]);
}

void detection::BoundingBox::createObjCenteredMembers()
{
  // Correct coordinates centering the object in the center of the bounding box
  Eigen::Affine3f t = translateCentroidToBoundingBox();
  min_pt_planar_ = pcl::transformPoint(min_pt_planar_centroid_, t);
  max_pt_planar_ = pcl::transformPoint(max_pt_planar_centroid_, t);

  // Rotate to be according the object frame reference system
  t = Eigen::AngleAxisf(pcl::deg2rad(90.0), Eigen::Vector3f(0.0, 1.0, 0.0)) * t;
  pcl::transformPointCloud(*planar_obj, *planar_obj, t);
}

Eigen::Affine3f detection::BoundingBox::translateCentroidToBoundingBox()
{
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  return t.translate(-planar_shift_);
}

Eigen::Vector3f detection::BoundingBox::getSizeWithExternHeight(float height)
{
  return Eigen::Vector3f(size_2D_[0], size_2D_[1], height);
}

/**
 * DO NOT USE. ONLY FOR BUILD_PLANAR
 * Change coordinates from kinect's to object's centroid.
 * First translate the object and then rotate it. Transforms are applied like right multiplication
 * so Transform = Identity * Rotation * Translate => Vector(final) = Transform * Vector(inicial)
 */
Eigen::Affine3f detection::BoundingBox::getKinectToCentroidTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // Rotate to objects coordinates
  transform.rotate(eigen_vectors_.transpose());
  // Translate to object's coordinates: Make the centroid be the origin
  return transform.translate(-centroid_2D_kinect_frame_.head<3>());
}

Eigen::Affine3f detection::BoundingBox::getCentroidToKinectTransform()
{
  return getKinectToCentroidTransform().inverse(Eigen::Affine);
}

Eigen::Quaternionf detection::BoundingBox::getInverseRotationQuaternion()
{
  return rotation_kinect_frame_.conjugate();
}

Eigen::Affine3f detection::BoundingBox::getObjBaseToKinectTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  return transform.translate(position_base_kinect_frame_).rotate(getRotationQuaternion());
}

Eigen::Affine3f detection::BoundingBox::getObjToKinectTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  return transform.translate(position_3D_kinect_frame_).rotate(getRotationQuaternion());
}

void things(pcl::visualization::PCLVisualizer &viz, bachelors_final_project::detection::BoundingBox *box)
{
  viz.addCube(Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf::Identity(),
              box->getSize3D()[0],
              box->getSize3D()[1],
              box->getSize3D()[2],
              "id");
}

void detection::BoundingBox::visualizeData()
{
  pcl::PointCloud<pcl::PointXYZRGB> brief;
  //brief.push_back(newPointXYZRGB(newPoint(size_2D_), 255, 0, 0));
  //brief.push_back(newPointXYZRGB(newPoint(size_2D_robot_frame_), 0, 255, 0));
  brief.push_back(newPointXYZRGB(min_pt_planar_centroid_, 255, 0, 0));
  brief.push_back(newPointXYZRGB(max_pt_planar_centroid_, 255, 0, 0));
  //brief.push_back(newPointXYZRGB(min_pt_planar_world_, 0, 0, 255));
  //brief.push_back(newPointXYZRGB(max_pt_planar_world_, 0, 0, 255));
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