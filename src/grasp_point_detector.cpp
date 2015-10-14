#include "grasp_point_detector.h"

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>

#include "utils.h"

using namespace pcl;

namespace bachelors_final_project
{

/*
 * Constructor
 */
detection::GraspPointDetector::GraspPointDetector(ros::NodeHandle &handle)
{
  bounding_box_.reset(new BoundingBox);
  //grasp_filter_.initializePublisher(handle);

  // Initialize pointers to point clouds
  object_cloud_.reset(new PointCloudT);
  transformed_cloud_.reset(new PointCloudT);
  projected_object_.reset(new PointCloudT);

  draw_bounding_box_ = false;
  draw_sampled_grasps_ = false;

}  // end GraspPointDetector()

void detection::GraspPointDetector::updateConfig(ParametersConfig &config)
{
  cfg = config;
}

void detection::GraspPointDetector::detect(const PointCloudTPtr &input_object, const pcl::ModelCoefficientsPtr &table_plane)
{
  clock_t beginCallback = clock();

  // If no cloud or no new images since last time, do nothing.
  if (input_object->size() == 0) return;

  object_cloud_ = input_object->makeShared();
  table_plane_ = boost::make_shared<ModelCoefficients>(*table_plane);

  kinect_frame_id_ = object_cloud_->header.frame_id;

  // Check if computation succeeded
  if (doProcessing())
    ROS_INFO("Detection callback took %gms\n\n", durationMillis(beginCallback));
}

bool detection::GraspPointDetector::doProcessing()
{
  ROS_INFO_ONCE("'Do Processing' called!");

  ProjectInliers<PointXYZ> proj;
  proj.setModelType(SACMODEL_PLANE);
  proj.setInputCloud(object_cloud_);
  proj.setModelCoefficients(table_plane_);
  proj.filter(*projected_object_);

  // projected_object_ lies in plane YZ with
  // a = blue (z) - b = green (y)

  clock_t begin = clock();
  boost::mutex::scoped_lock bounding_box_lock(update_bounding_box_mutex_);
  computeBoundingBox(projected_object_, bounding_box_);

  // Find all the samples poses
  sampler.sampleGraspingPoses(bounding_box_);
  draw_sampled_grasps_ = true;
  bounding_box_lock.unlock();
  ROS_INFO("Grasping sampling  took %gms", durationMillis(begin));

  begin = clock();
  // Configure filter
  //grasp_filter_.configure(kinect_frame_id_, bounding_box_, table_plane_);
  // Remove infeasible ones
  //grasp_filter_.filterGraspingPoses(sampler.getSideGrasps(), sampler.getTopGrasps());
  //ROS_INFO("Grasping filtering took %gms", durationMillis(begin));

  return true;
}

/**
 * 1) compute the centroid (c0, c1, c2) and the normalized covariance
 * 2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
 * 3) move the points in that RF
   - note: the transformation given by the rotation_ matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
 * 4) compute the max, the min and the center of the diagonal
 * 5) given a box centered at the origin with size (max_pt_.x - min_pt_.x, max_pt_.y - min_pt_.y, max_pt_.z - min_pt_.z)
   the transformation you have to apply is
   Rotation = (e0, e1, e0 X e1) & Translation = Rotation * center_diag + (c0, c1, c2)
 */
void detection::GraspPointDetector::computeBoundingBox(PointCloudTPtr &obj_cloud, BoundingBoxPtr &bounding_box)
{
  // Compute princcloudipal direction
  Eigen::Vector4f sensor_centroid;
  pcl::compute3DCentroid(*obj_cloud, sensor_centroid);
  Eigen::Matrix3f sensor_covariance;
  computeCovarianceMatrixNormalized(*obj_cloud, sensor_centroid, sensor_covariance);

  // Compute eigen vectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(sensor_covariance);
  Eigen::Matrix3f sensor_eigen_vectors = eigen_solver.eigenvectors();
  // Third component is orthogonal to principal axes
  sensor_eigen_vectors.col(2) = sensor_eigen_vectors.col(0).cross(sensor_eigen_vectors.col(1));

  // Move the points to the that reference frame
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  // Change coordinates from kinect's to object's
  // Rotate to objects coordinates
  Eigen::Matrix3f rotation_to_obj_coords(sensor_eigen_vectors.transpose());
  // Translate to ojb coordinates: Make the centroid be the origin
  transform.translate(-1.f * (rotation_to_obj_coords * sensor_centroid.head<3>()));
  transform.rotate(rotation_to_obj_coords);
  pcl::transformPointCloud(*obj_cloud, *transformed_cloud_, transform);

  PointT origin_min_pt, origin_max_pt;
  getMinMax3D(*transformed_cloud_, origin_min_pt, origin_max_pt);
  const Eigen::Vector3f origin_bounding_box_center =
      0.5f * (origin_max_pt.getVector3fMap() + origin_min_pt.getVector3fMap());

  // Final transform: back to object coordinates
  const Eigen::Quaternionf rotation_to_sensor_coords(sensor_eigen_vectors);
  const Eigen::Vector3f translation_to_sensor_coords = sensor_eigen_vectors * origin_bounding_box_center +
                                                       sensor_centroid.head<3>();

  transform = Eigen::Affine3f::Identity();
  transform.translate(-origin_bounding_box_center);
  pcl::transformPointCloud(*transformed_cloud_, *transformed_cloud_, transform);

  PointT origin_min_3d_pt, origin_max_3d_pt;
  getMinMax3D(*object_cloud_, origin_min_3d_pt, origin_max_3d_pt);
  double heigth_3D = (origin_max_3d_pt.x - origin_min_3d_pt.x) / 2.0;

  bounding_box->initialize(origin_min_pt, origin_max_pt, rotation_to_sensor_coords, translation_to_sensor_coords,
                           sensor_centroid, sensor_eigen_vectors, origin_bounding_box_center, heigth_3D);

  draw_bounding_box_ = true;
}


PointCloudTPtr detection::GraspPointDetector::getSampledSideGrasps()
{
  return sampler.getSideGrasps();
}

PointCloudTPtr detection::GraspPointDetector::getSampledTopGrasps()
{
  return sampler.getTopGrasps();
}

} // namespace bachelors_final_project
