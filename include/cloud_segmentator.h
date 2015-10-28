#ifndef CLOUD_SEGMENTATOR_H
#define CLOUD_SEGMENTATOR_H

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <boost/thread/pthread/mutex.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

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
  CloudSegmentator(ros::NodeHandle nh, tf2_ros::TransformListener &tf_listener);

  void updateConfig(ParametersConfig &config);

  //! Callback function for subscriber.
  void sensorCallback(const CloudConstPtr &sensorInput);

  bool doProcessing(const CloudPtr &input);

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
  bool fitPlaneFromNormals(const CloudPtr &input, CloudNormalPtr &normals,
                           pcl::ModelCoefficientsPtr &coefficients, pcl::PointIndicesPtr &inliers);

  bool computeNormalsEfficiently(const CloudPtr &sensor_cloud, CloudNormalPtr &cloud_normals_);

  void computeTableConvexHull(const CloudPtr &projectedTableCloud, CloudPtr &tableConvexHull);

  bool extractCloudOverTheTable(const CloudPtr &sensor_cloud, const CloudPtr &tableConvexHull,
                                pcl::PointIndicesPtr &indices_over_table);

  void cropOrganizedPointCloud(const CloudPtr &cloud_in,
                               CloudPtr &cropped_cloud);

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
  bool clusterObjects(const CloudPtr &cloud_in,
                      const pcl::PointIndicesPtr &indices);

  CloudPtr getCluster(size_t index);

  const pcl::ModelCoefficientsPtr getTable();

  void setPlaneAxis(pcl::SACSegmentationFromNormals<Point, Normal> &seg);

  bool clearSegmentation(FailedLevel failed_level);

  void pointCloudFromIndices(const CloudPtr &input, pcl::PointIndicesPtr &inliers, CloudPtr &output);

  void normalPointCloudFromIndices(const CloudNormalPtr &input, pcl::PointIndicesPtr &inliers,
                                   CloudNormalPtr &output);

  bool setProcessedCloud(bool state);

  bool noNewProcessedData();

  const CloudPtr getTableCloud();

  std::vector<CloudPtr> &getClusters()
  {
    return clusters_vector_;
  }

  bool point_clouds_updated_;
  bool plane_updated_;
  bool cloud_over_table_updated_;

  bool clusters_updated_;

  boost::mutex update_normals_mutex_;
  ros::Publisher pub_planar_, pub_objects_;

  CloudPtr sensor_cloud_;
  CloudPtr cropped_cloud_;
  CloudPtr plane_cloud_;
  CloudPtr cropped_cloud_base_frame;
  CloudPtr cloud_over_table_;
  pcl::PointIndicesPtr indices_over_table_;
  CloudNormalPtr cloud_normals_;

  pcl::SACSegmentationFromNormals<Point, Normal> sac_segmentation_;
  pcl::IntegralImageNormalEstimation<Point, Normal> normal_estimation_;
  pcl::ExtractIndices<Point> extract_;
  pcl::ExtractIndices<Normal> extract_normals_;
  pcl::SACSegmentation<Point>::SearchPtr search_;
  pcl::ConvexHull<Point> convex_hull_;
  pcl::ExtractPolygonalPrismData<Point> extract_prism_;
  pcl::EuclideanClusterExtraction<Point> euclidean_clustering_;

  std::vector<CloudPtr> clusters_vector_;

  pcl::ModelCoefficientsPtr table_coefficients_;

  ParametersConfig cfg;


  uint32_t last_seen_seq_;

  Point plane_normal_base_frame_, plane_normal_kinect_frame_;
  Point normal_base_frame_reconstructed_;

  tf2_ros::TransformListener &tf_listener_;

  CloudPtr projected_table_cloud_;
  bool processed_cloud_;
};

} // namespace segmentation
} // namespace bachelors_final_project

#endif // CLOUD_SEGMENTATOR_H

