#ifndef DATAHANDLER
#define DATAHANDLER

// ROS includes
#include <ros/ros.h>

// PCL specific includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>

// Boost
#include <boost/thread/pthread/mutex.hpp>

// Auto-generated from cfg/ directory.
#include <bachelors_final_project/ParametersConfig.h>

namespace
{
class NodeHandle;
class Publisher;
}

namespace bachelors_final_project
{
namespace segmentation
{

class DataHandler
{
public:
  //! Constructor.
  DataHandler(ros::NodeHandle nh);

  //! Destructor.
  ~DataHandler();

  void measureCallback(clock_t begin);

  //! Callback function for subscriber.
  void sensorCallback(const pcl::PCLPointCloud2::ConstPtr &sensorInput);

  bool doProcessing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);

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
  bool fitPlaneFromNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointCloud<pcl::Normal>::Ptr &normals,
                           pcl::ModelCoefficients::Ptr &coefficients, pcl::PointIndices::Ptr &inliers);

  void extractPlaneCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input, pcl::PointIndices::Ptr &inliers);

  void computeNormalsEfficiently(const pcl::PointCloud<pcl::PointXYZ>::Ptr &sensor_cloud,
                                 pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals_);

  void projectOnPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &sensor_cloud,
                      const pcl::ModelCoefficients::Ptr &tableCoefficients,
                      const pcl::PointIndices::Ptr &tableInliers,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &projectedTableCloud);

  void computeTableConvexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &projectedTableCloud,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr &tableConvexHull);

  bool extractCloudOverTheTable(const pcl::PointCloud<pcl::PointXYZ>::Ptr &sensor_cloud,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &tableConvexHull,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudOverTheTable);

  pcl::PointCloud<pcl::PointXYZ>::Ptr gaussianSmoothing(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudInput,
                                                        pcl::PointCloud<pcl::PointXYZ>::Ptr &smoothed_cloud_);

  void cropOrganizedPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudInput,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &croppedCloud);

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
  void clusterObjects(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudOverTheTable);

  bool point_clouds_updated_;
  bool plane_updated_;
  bool cloud_over_table_updated_;
  bool clusters_updated_;
  boost::mutex update_normals_mutex_;

  ros::Publisher pub_planar_, pub_objects_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_over_table_;

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster_vector_;

  pcl::ModelCoefficients::Ptr table_coefficients_;

  bachelors_final_project::ParametersConfig cfg;

  uint32_t last_seen_seq_;

  void updateConfig(bachelors_final_project::ParametersConfig &config);
};

} // namespace segmentation
} // namespace bachelors_final_project

#endif // DATAHANDLER

