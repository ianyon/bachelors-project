//
// Created by ian on 10/22/15.
//

#ifndef BACHELORS_FINAL_PROJECT_BOUNDING_BOX_H
#define BACHELORS_FINAL_PROJECT_BOUNDING_BOX_H

#include <Eigen/Eigenvalues>
#include "definitions.h"


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
public:
  BoundingBox(std::string obj_frame);

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

  Eigen::Affine3f getWorldToObjectCentroidTransform();

  Eigen::Vector3f worldCoordsBoundingBoxPose();

  inline double getXLength()
  {
    return heigth_3D_;
  }

  inline double getYLength()
  {
    return max_pt_planar_centroid_.y - min_pt_planar_centroid_.y;
  }

  inline double getZLength()
  {
    return max_pt_planar_centroid_.z - min_pt_planar_centroid_.z;
  }

  inline double getXLengthWorldCoords()
  {
    return max_pt_planar_centroid_.z - min_pt_planar_centroid_.z;
  }

  inline double getYLengthWorldCoords()
  {
    return max_pt_planar_centroid_.y - min_pt_planar_centroid_.y;
  }

  inline double getZLengthWorldCoords()
  {
    return heigth_3D_;
  }

  Eigen::Affine3f translateCentroidToBoundingBox(Eigen::Affine3f transform);

  void createBoundingBoxCenteredMembers();

  void createWorldCenteredMembers();

  void createSize();

  const Eigen::Vector3f &getSizePlanarFootprint();

  const Point getPlanarWorldPose();

  Point computeFootprintPosePosition(tf2_ros::TransformListener &tf_listener);

  void visualizeData();

  // Minimum and maximum bounding box points relative to named coordinate system
  Point min_pt_planar_centroid_, max_pt_planar_centroid_,
      min_pt_planar_world_, max_pt_planar_world_,
      min_pt_planar_, max_pt_planar_;
  Point pose_planar_world_;
  std::string kinect_frame_;
  // rotation_kinect_frame_ represents the eigen vectors as a rotation matrix
  Eigen::Quaternionf rotation_kinect_frame_;
  Eigen::Vector4f world_coords_planar_centroid_;
  Eigen::Matrix3f eigen_vectors_;
  Eigen::Vector3f planar_shift_, position_3D_kinect_frame_, position_2D_kinect_frame_, size_planar_, size_planar_footprint_;
  double heigth_3D_;
  CloudPtr planar_obj;
  const std::string OBJ_FRAME;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver;

  CloudPtr &getPlanarObj()
  {
    return planar_obj;
  }

  geometry_msgs::Pose computeFootprintPose(tf2_ros::TransformListener &tf_listener);

  void broadcastFrameUpdate(tf2_ros::TransformBroadcaster broadcaster, Eigen::Vector3f &position);
  void broadcast2DFrameUpdate(tf2_ros::TransformBroadcaster broadcaster);

  void build3DAndPublishFrame(CloudPtr &world_coords_obj, tf2_ros::TransformBroadcaster broadcaster);

  Eigen::Affine3f getWorldToObjectTransform();

  Eigen::Affine3f getObjectToWorldTransform();

};

typedef boost::shared_ptr<BoundingBox> BoundingBoxPtr;

} // namespace detection
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_BOUNDING_BOX_H
