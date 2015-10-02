#include "grasp_point_detector.h"

#include <boost/shared_ptr.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>

#include "utils.h"

using namespace pcl;

namespace bachelors_final_project
{

/*
 * Constructor
 */
detection::GraspPointDetector::GraspPointDetector()
{
  // Initialize pointers to point clouds
  object_cloud_.reset(new PointCloudT);
  transformed_cloud_.reset(new PointCloudT);
  projected_object_.reset(new PointCloudT);

  draw_bounding_box_ = false;

}  // end GraspPointDetector()

void detection::GraspPointDetector::updateConfig(ParametersConfig &config)
{
  cfg = config;
}

void detection::GraspPointDetector::detect(const PointCloudTPtr &input_object, const ModelCoefficientsPtr &table_plane)
{
  clock_t beginCallback = clock();

  // If no cloud or no new images since last time, do nothing.
  if (input_object->size() == 0) return;

  object_cloud_ = input_object->makeShared();
  //table_plane_.reset(new ModelCoefficients(*table_plane));
  table_plane_ = boost::make_shared<ModelCoefficients>(*table_plane);

  // Check if computation succeeded
  if (doProcessing())
    ROS_INFO("Detection callback took %gms\n\n", durationMillis(beginCallback));
}

bool detection::GraspPointDetector::doProcessing()
{
  ROS_INFO_ONCE("'Do Processing' called!");

  /*ProjectInliers<PointXYZ> proj;
  proj.setModelType(SACMODEL_PLANE);
  proj.setInputCloud(object_cloud_);
  proj.setModelCoefficients(table_plane_);
  proj.filter(*projected_object_);*/

  // TODO: check if we need planar object or 3D one
  boundingBox(object_cloud_, &bounding_box_);

  // Find all the samples poses
  sampleGraspingPoses(bounding_box_, &sampled_grasps_);

  return true;
}

/**
 * 1) compute the centroid (c0, c1, c2) and the normalized covariance
 * 2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
 * 3) move the points in that RF
   - note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
 * 4) compute the max, the min and the center of the diagonal
 * 5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z)
   the transformation you have to apply is
   Rotation = (e0, e1, e0 X e1) & Translation = Rotation * center_diag + (c0, c1, c2)
 */
void detection::GraspPointDetector::boundingBox(PointCloudTPtr &cloud, BoundingBox *bounding_box)
{
  // Compute principal direction
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cloud, centroid, covariance);

  // Compute eigen vectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
  Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
  // Third component is orthogonal to principal axes
  eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));

  // Move the points to the that reference frame
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Matrix3f rotation_matrix(eigen_vectors.transpose());
  transform.translation(-1.f * (rotation_matrix * centroid.head<3>()) );
  transform.rotate(rotation_matrix);
  pcl::transformPointCloud(*cloud, *transformed_cloud_, transform);

  PointT min_pt, max_pt;
  getMinMax3D(*transformed_cloud_, min_pt, max_pt);
  const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  // final transform
  const Eigen::Quaternionf rotation(eigen_vectors);
  const Eigen::Vector3f translation = eigen_vectors * mean_diag + centroid.head<3>();

  bounding_box->initialize(min_pt, max_pt, rotation, translation, centroid, eigen_vectors, mean_diag);

  draw_bounding_box_ = true;
}


} // namespace bachelors_final_project
