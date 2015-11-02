//
// Created by ian on 10/22/15.
//

#ifndef BACHELORS_FINAL_PROJECT_BOUNDING_BOX_H
#define BACHELORS_FINAL_PROJECT_BOUNDING_BOX_H

#include <Eigen/Eigenvalues>
#include "definitions.h"

#include <tf/transform_broadcaster.h>

#include <pcl/visualization/cloud_viewer.h>

namespace bachelors_final_project
{
namespace detection
{

/**
 * All member variables are in the object's coordinates unless their name indicate the opossite
 */
class BoundingBox
{
  // Minimum and maximum bounding box points relative to named coordinate system
  Point min_pt_planar_centroid_, max_pt_planar_centroid_,
      min_pt_planar_world_, max_pt_planar_world_,
      min_pt_planar_, max_pt_planar_;

  // rotation_kinect_frame_ represents the eigen vectors as a rotation matrix
  Eigen::Quaternionf rotation_kinect_frame_;

  Eigen::Affine3f getKinectToCentroidTransform();

public:
  BoundingBox(std::string obj_frame);

  void computeAndPublish(CloudPtr &world_obj_, pcl::ModelCoefficientsPtr table_plane_,
                         tf::TransformBroadcaster tf_broadcaster);

  /**
 * 1) compute the centroid (c0, c1, c2) and the normalized covariance
 * 2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
 * 3) move the points in that RF
   - note: the transformation given by the rotation_kinect_frame_ matrix (e0, e1, e0 X e1) & (c0, c1, c2)
     must be inverted
 * 4) compute the max, the min and the center of the diagonal
 * 5) given a box centered at the origin with size (max_point_.x - min_point_.x, max_point_.y - min_point_.y,
   *  max_point_.z - min_point_.z) the transformation you have to apply is
   Rotation = (e0, e1, e0 X e1) & Translation = Rotation * center_diag + (c0, c1, c2)
 */
  void buildPlanar(CloudPtr &world_coords_planar_obj);

  inline double getXLength()
  { return size_3D_[0]; }

  inline double getYLength()
  { return max_pt_planar_centroid_.y - min_pt_planar_centroid_.y; }

  inline double getZLength()
  { return max_pt_planar_centroid_.z - min_pt_planar_centroid_.z; }

  inline double getXLengthWorldCoords()
  { return max_pt_planar_centroid_.z - min_pt_planar_centroid_.z; }

  inline double getYLengthWorldCoords()
  { return max_pt_planar_centroid_.y - min_pt_planar_centroid_.y; }

  inline double getHeight()
  { return size_3D_[2]; }

  inline Eigen::Vector3f getSize3D() const
  { return size_3D_; }

  inline Eigen::Vector2f getSize2D() const
  { return size_2D_; }

  inline float getMayorAxisSize2D()
  { return size_2D_[0]; }

  inline float getMinorAxisSize2D()
  { return size_2D_[1]; }

  inline pcl::PointXY getMin2D()
  {
    pcl::PointXY p;
    p.x = min_pt_planar_.z;
    p.y = min_pt_planar_.y;
    return p;
  }

  inline pcl::PointXY getMax2D()
  {
    pcl::PointXY p;
    p.x = max_pt_planar_.z;
    p.y = max_pt_planar_.y;
    return p;
  }

  inline Point getMin3D()
  { return Point(getMin2D().x, getMin2D().y, -(float)getHeight()/2); }

  inline Point getMax3D()
  { return Point(getMax2D().x, getMax2D().y, (float)getHeight()/2); }

  inline Eigen::Quaternionf getRotationQuaternion()
  { return rotation_kinect_frame_; }

  inline tf::Quaternion getRotationQuaternionTF()
  { return tf::Quaternion(rotation_kinect_frame_.x(), rotation_kinect_frame_.y(),
                          rotation_kinect_frame_.z(), rotation_kinect_frame_.w()); }

  inline CloudPtr &getPlanarObj()
  { return planar_obj; }

  Eigen::Affine3f translateCentroidToBoundingBox(Eigen::Affine3f transform);

  void createObjCenteredMembers();

  void createKinectCenteredMembers();

  void create2DSize();

  Eigen::Quaternionf getInverseRotationQuaternion();

  Eigen::Vector3f getSizeWithExternHeight(float height);

  Point computePosition2DRobotFrame(tf::TransformListener &tf_listener);

  Eigen::Quaternionf createRotationQuaternion2D(Eigen::Matrix3f eigen_vectors);

  void visualizeData();

  std::string kinect_frame_;
  Eigen::Vector4f centroid_2D_kinect_frame_;
  Eigen::Matrix3f eigen_vectors_;
  Eigen::Vector3f planar_shift_, position_3D_kinect_frame_, position_base_kinect_frame_;

  // Planar size. Index 0 is 'z' and index 1 is 'y'
  Eigen::Vector2f size_2D_;
  Eigen::Vector3f size_3D_;
  CloudPtr planar_obj;
  const std::string OBJ_FRAME;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver;

  geometry_msgs::Pose computePose3DRobotFrame(tf::TransformListener &tf_listener);

  void broadcastFrameUpdate(tf::TransformBroadcaster broadcaster, Eigen::Vector3f &position);

  void broadcast2DFrameUpdate(tf::TransformBroadcaster broadcaster);

  void build3DAndPublishFrame(CloudPtr &world_coords_obj, tf::TransformBroadcaster broadcaster);

  Eigen::Affine3f getObjToKinectBaseTransform();

  Eigen::Affine3f getObjToKinectTransform();
};

typedef boost::shared_ptr<BoundingBox> BoundingBoxPtr;

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_BOUNDING_BOX_H
