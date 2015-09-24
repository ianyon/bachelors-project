#ifndef DATAHANDLER
#define DATAHANDLER

// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <bachelors_final_project/ParametersConfig.h>

// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

//Common
#include <pcl/common/gaussian.h>
#include <pcl/common/io.h>
#include <pcl/common/eigen.h>

// Filters
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/convolution_3d.h>

// Sample consensus
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include <pcl/io/pcd_io.h>

//#include <pcl/kdtree/kdtree.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include "../include/utils.h"

using namespace pcl;
using namespace std;
using namespace utils;

class DataHandler
{
public:
    //! Constructor.
    DataHandler(ros::NodeHandle nh);

    //! Destructor.
    ~DataHandler();

    void measureCallback(clock_t begin);

    //! Callback function for subscriber.
    void sensorCallback (const PCLPointCloud2::ConstPtr& sensorInput);

    bool doProcessing(const PointCloud<PointXYZ>::Ptr &input);
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
    bool fitPlaneFromNormals (const PointCloud<PointXYZ>::Ptr &input, PointCloud<Normal>::Ptr &normals,
                              ModelCoefficients::Ptr &coefficients, PointIndices::Ptr &inliers);

    void extractPlaneCloud (const PointCloud<PointXYZ>::Ptr &input, PointIndices::Ptr &inliers);

    void computeNormalsEfficiently(const PointCloud<PointXYZ>::Ptr &sensor_cloud,
                                   PointCloud<Normal>::Ptr &cloud_normals_);

    void projectOnPlane(const PointCloud<PointXYZ>::Ptr &sensor_cloud, const ModelCoefficients::Ptr &tableCoefficients,
                        const PointIndices::Ptr &tableInliers, PointCloud<PointXYZ>::Ptr &projectedTableCloud);

    void computeTableConvexHull(const PointCloud<PointXYZ>::Ptr& projectedTableCloud,
                                PointCloud<PointXYZ>::Ptr& tableConvexHull);

    bool extractCloudOverTheTable(const PointCloud<PointXYZ>::Ptr& sensor_cloud,
                                  const PointCloud<PointXYZ>::Ptr& tableConvexHull,
                                  PointCloud<PointXYZ>::Ptr& cloudOverTheTable);

    PointCloud<PointXYZ>::Ptr gaussianSmoothing(const PointCloud<PointXYZ>::Ptr &cloudInput,
                                                PointCloud<PointXYZ>::Ptr &smoothed_cloud_);

    void cropOrganizedPointCloud(const PointCloud<PointXYZ>::Ptr &cloudInput, PointCloud<PointXYZ>::Ptr &croppedCloud);

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
    void clusterObjects (const PointCloud<PointXYZ>::Ptr & cloudOverTheTable);

    bool point_clouds_updated_;
    bool plane_updated_;    
    bool cloud_over_table_updated_;
    bool clusters_updated_;
  boost::mutex update_normals_mutex_;

    ros::Publisher pub_smoothed_, pub_planar_, pub_objects_;

    PointCloud<PointXYZ>::Ptr sensor_cloud_;
    PointCloud<PointXYZ>::Ptr smoothed_cloud_;
    PointCloud<PointXYZ>::Ptr plane_cloud_;
    PointCloud<Normal>::Ptr cloud_normals_;
    PointCloud<PointXYZ>::Ptr cloud_over_table_;

    vector<PointCloud<PointXYZ>::Ptr > cloud_cluster_vector_;

    ModelCoefficients::Ptr table_coefficients_;

    // Cropping
    double scale;
    int xTranslate, yTranslate;

    // Gaussian Smoothing
    double gaussianSigma;
    double gaussianSearchRadius;

    // Compute normals efficiently
    int normalEstimationMethod;
    double maxDepthChangeFactor;
    bool useDepthDependentSmoothing;
    double normalSmoothingSize;

    // Fit Plane From Normals
    double normalDistanceWeight;
    double originDistance;
    int maxIterations;
    double distanceThreshold;
    bool optimizeCoefficients;
    double probability;
    double sampleMaxDistance;
    bool useSpecificPlane;
    double planeX, planeY, planeZ;
    double epsAngle;

    // Cloud Over The Table
    double minHeight, maxHeight;

    // Euclidean Cluster
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;


  uint32_t last_seen_seq_;
};

#endif // DATAHANDLER

