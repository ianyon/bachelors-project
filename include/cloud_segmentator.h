#ifndef CLOUD_SEGMENTATOR_H
#define CLOUD_SEGMENTATOR_H

#include <ros/ros.h>

#include <boost/thread/pthread/mutex.hpp>

#include <pcl_ros/point_cloud.h>

#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

#include "definitions.h"
#include <bachelors_final_project/ParametersConfig.h>
#include <pcl/segmentation/sac_segmentation.h>

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
  void sensorCallback(const PointCloudTConstPtr &sensorInput);

  bool doProcessing(const PointCloudTPtr &input);

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
  bool fitPlaneFromNormals(const PointCloudTPtr &input, PointCloudNormalPtr &normals,
                           pcl::ModelCoefficientsPtr &coefficients, pcl::PointIndicesPtr &inliers);

  void computeNormalsEfficiently(const PointCloudTPtr &sensor_cloud, PointCloudNormalPtr &cloud_normals_);

  void projectOnPlane(const PointCloudTPtr &sensor_cloud, const pcl::ModelCoefficientsPtr &tableCoefficients,
                      const pcl::PointIndicesPtr &tableInliers, PointCloudTPtr &projectedTableCloud);

  void computeTableConvexHull(const PointCloudTPtr &projectedTableCloud, PointCloudTPtr &tableConvexHull);

  bool extractCloudOverTheTable(const PointCloudTPtr &sensor_cloud, const PointCloudTPtr &tableConvexHull,
                                PointCloudTPtr &cloud_over_table);

  void cropOrganizedPointCloud(const PointCloudTPtr &cloud_in,
                               PointCloudTPtr &cropped_cloud);

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
  void clusterObjects(const PointCloudTPtr &cloudOverTheTable);

  PointCloudTPtr getCluster(size_t index);

  const pcl::ModelCoefficientsPtr getTable();

  void setPlaneAxis(pcl::SACSegmentationFromNormals<PointT, Normal> &seg);

  void clearSegmentation();

  bool point_clouds_updated_;
  bool plane_updated_;
  bool cloud_over_table_updated_;

  bool clusters_updated_;

  boost::mutex update_normals_mutex_;
  ros::Publisher pub_planar_, pub_objects_;

  PointCloudTPtr sensor_cloud_;
  PointCloudTPtr cropped_cloud_;
  PointCloudTPtr plane_cloud_;
  PointCloudTPtr cropped_cloud_base_frame;
  PointCloudTPtr cloud_over_table_;

  PointCloudNormalPtr cloud_normals_;

  std::vector<PointCloudTPtr> cloud_cluster_vector_;

  pcl::ModelCoefficientsPtr table_coefficients_;

  ParametersConfig cfg;

  uint32_t last_seen_seq_;

  PointT plane_normal_base_frame_, plane_normal_kinect_frame_;
};

} // namespace segmentation
} // namespace bachelors_final_project

#endif // CLOUD_SEGMENTATOR_H

