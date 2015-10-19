#ifndef CLOUD_SEGMENTATOR_H
#define CLOUD_SEGMENTATOR_H

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <boost/thread/pthread/mutex.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/PointIndices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "definitions.h"
#include <bachelors_final_project/ParametersConfig.h>

namespace bachelors_final_project
{
namespace segmentation
{

class CloudSegmentator
{
public:
  //! Constructor.
  CloudSegmentator(ros::NodeHandle nh);

  void updateConfig(ParametersConfig &config);

  //! Callback function for subscriber.
  void sensorCallback(const PointCloudConstPtr &sensorInput);

  bool doProcessing(const PointCloudPtr &input);

  void execute();

  /* Use SACSegmentation to find the dominant plane in the scene
 * Inputs:
 *   input
 *     The input point cloud
 *   distanceThreshold
 *     The inlier/outlier threshold.  Points within this distance
 *     from the hypothesized plane are scored as inliers.
 * Return: A pointer to the ModelCoefficients (i.e., the 4 coefficients of the plane,
 *         represented in c0*x + c1*y + c2*z + c3 = 0 form)
 */
  bool fitPlaneFromNormals(const PointCloudPtr &input, PointCloudNormalPtr &normals,
                           pcl::ModelCoefficientsPtr &coefficients, pcl::PointIndicesPtr &inliers);

  bool computeNormalsEfficiently(const PointCloudPtr &sensor_cloud, PointCloudNormalPtr &cloud_normals_);

  void projectOnPlane(const PointCloudPtr &sensor_cloud, const pcl::ModelCoefficientsPtr &tableCoefficients,
                      const pcl::PointIndicesPtr &tableInliers, PointCloudPtr &projectedTableCloud);

  void computeTableConvexHull(const PointCloudPtr &projectedTableCloud, PointCloudPtr &tableConvexHull);

  bool extractCloudOverTheTable(const PointCloudPtr &sensor_cloud, const PointCloudPtr &tableConvexHull,
                                pcl::PointIndicesPtr &indices_over_table);

  void cropOrganizedPointCloud(const PointCloudPtr &cloud_in,
                               PointCloudPtr &cropped_cloud);

  /* Use EuclidieanClusterExtraction to group a cloud into contiguous clusters
 * Inputs:
 *   input
 *     The input point cloud
 *   cluster_tolerance
 *     The maximum distance between neighboring points in a cluster [cm]
 *   min/max_cluster_size
 *     The minimum and maximum allowable cluster sizes
 * Return (by reference): a vector of PointIndices containing the points indices in each cluster
 */
  bool clusterObjects(const PointCloudPtr &cloud_in,
                      const pcl::PointIndicesPtr &indices);

  PointCloudPtr getCluster(size_t index);

  const pcl::ModelCoefficientsPtr getTable();

  void setPlaneAxis(pcl::SACSegmentationFromNormals<Point, Normal> &seg);

  bool clearSegmentation(FailedLevel failed_level);

  bool point_clouds_updated_;
  bool plane_updated_;
  bool cloud_over_table_updated_;

  bool clusters_updated_;

  boost::mutex update_normals_mutex_;
  ros::Publisher pub_planar_, pub_objects_;

  PointCloudPtr sensor_cloud_;
  PointCloudPtr cropped_cloud_;
  PointCloudPtr plane_cloud_;
  PointCloudPtr cropped_cloud_base_frame;
  PointCloudPtr cloud_over_table_;
  pcl::PointIndicesPtr indices_over_table_;

  PointCloudNormalPtr cloud_normals_;

  std::vector<PointCloudPtr> clusters_vector_;

  pcl::ModelCoefficientsPtr table_coefficients_;

  ParametersConfig cfg;

  uint32_t last_seen_seq_;

  tf::TransformListener tf_listener_;
  Point plane_normal_base_frame_, plane_normal_kinect_frame_;
  Point normal_base_frame_reconstructed_;
};

} // namespace segmentation
} // namespace bachelors_final_project

#endif // CLOUD_SEGMENTATOR_H

