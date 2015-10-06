#include <viewer_spawner.h>
#include <dynamic_reconfigure/server.h>
#include "cloud_segmentator.h"

#include <pcl/filters/filter.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/integral_image_normal.h>

#include <pcl/surface/convex_hull.h>

#include "utils.h"

using namespace pcl;

namespace bachelors_final_project
{

/*
 * Constructor
 */
segmentation::CloudSegmentator::CloudSegmentator(ros::NodeHandle nh)
{
  // Create a ROS publisher
  pub_planar_ = nh.advertise<PointCloudT>("planar", 1);
  pub_objects_ = nh.advertise<PointCloudT>("objects", 1);

  // Initialize pointers to point clouds
  sensor_cloud_.reset(new PointCloudT);
  smoothed_cloud_.reset(new PointCloudT);
  plane_cloud_.reset(new PointCloudT);
  cloud_normals_.reset(new PointCloud<Normal>);
  table_coefficients_.reset(new ModelCoefficients());
  cloud_over_table_.reset(new PointCloudT);

  // Initialize flags
  plane_updated_ = false;
  point_clouds_updated_ = false;
  cloud_over_table_updated_ = false;
  clusters_updated_ = false;

  last_seen_seq_ = 0;

}  // end CloudSegmentator()

void segmentation::CloudSegmentator::updateConfig(ParametersConfig &config)
{
  cfg = config;
}

void segmentation::CloudSegmentator::sensorCallback(const PointCloudTConstPtr &sensorInput)
{
  ROS_INFO_ONCE("Callback Called");
  sensor_cloud_ = sensorInput->makeShared();
}

void segmentation::CloudSegmentator::cropOrganizedPointCloud(const PointCloudTPtr &cloudInput,
                                                             PointCloudTPtr &croppedCloud)
{
  clock_t begin = clock();
  // Kinect is 640/480
  int width = (int) floor(64 * cfg.scaleParam),
      height = (int) floor(48 * cfg.scaleParam);
  int init_col = (int) floor(fmax(cloudInput->width / 2 - width / 2 + cfg.xTranslateParam, 0)),
      init_row = (int) floor(fmax(cloudInput->height / 2 - height / 2 + cfg.yTranslateParam, 0));

  // Make que dimensions of the cloud be according the size of the vector
  croppedCloud->width = (uint32_t) width;
  croppedCloud->height = (uint32_t) height;
  // Change the size of the pointcloud
  croppedCloud->points.resize((unsigned long) (width * height));
  croppedCloud->sensor_origin_ = cloudInput->sensor_origin_;
  croppedCloud->sensor_orientation_ = cloudInput->sensor_orientation_;

  for (unsigned int u = 0; u < width; u++)
  {
    for (unsigned int v = 0; v < height; v++)
    {
      // We can't use croppedCloud->push_back because it breaks the organization
      croppedCloud->at(u, v) = cloudInput->at(init_col + u, init_row + v);
    }
  }

  croppedCloud->is_dense = cloudInput->is_dense;

  ROS_DEBUG("PointCloud cropping took %gms", durationMillis(begin));
  ROS_DEBUG("Cropped from %lu to %lu", cloudInput->points.size(), croppedCloud->points.size());
}

PointCloudTPtr segmentation::CloudSegmentator::gaussianSmoothing(const PointCloudTPtr &cloudInput,
                                                                 PointCloudTPtr &smoothed_cloud_)
{
  clock_t begin = clock();
  //Set up the Gaussian Kernel
  filters::GaussianKernel<PointXYZ, PointXYZ>::Ptr kernel(new filters::GaussianKernel<PointXYZ, PointXYZ>());
  kernel->setSigma((float) cfg.gaussianSigmaParam);
  kernel->setThresholdRelativeToSigma(3);

  //Set up the KDTree
  search::KdTree<PointXYZ>::Ptr kdTree(new search::KdTree<PointXYZ>);
  kdTree->setInputCloud(cloudInput);

  //Set up the Convolution Filter
  filters::Convolution3D<PointXYZ, PointXYZ, filters::GaussianKernel<PointXYZ, PointXYZ> > convolution;
  convolution.setKernel(*kernel);
  convolution.setInputCloud(cloudInput);
  convolution.setSearchMethod(kdTree);
  convolution.setRadiusSearch(cfg.gaussianSearchRadiusParam);
  convolution.convolve(*smoothed_cloud_);

  ROS_DEBUG("Gaussian Smoothing took %gms", durationMillis(begin));

  return smoothed_cloud_;
}


void segmentation::CloudSegmentator::computeNormalsEfficiently(const PointCloudTPtr &sensor_cloud,
                                                               PointCloudNormalPtr &cloud_normals)
{
  clock_t begin = clock();
  IntegralImageNormalEstimation<PointXYZ, Normal> ne;

  switch (cfg.normalEstimationMethodParam)
  {
    case 1:
      ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
      break;
    case 2:
      ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
      break;
    case 3:
      ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
      break;
    case 4:
      ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
      break;
    default:
      ROS_ERROR("Wrong Normal estimation method. Parameter error");
      ros::shutdown();
      return;
  }

  // TODO: Analyze use multithreading OMP Classes
  ne.setMaxDepthChangeFactor((float) cfg.maxDepthChangeFactorParam);
  ne.setDepthDependentSmoothing(cfg.useDepthDependentSmoothingParam);
  ne.setNormalSmoothingSize((float) cfg.normalSmoothingSizeParam);
  ne.setInputCloud(sensor_cloud);
  //ne.setKSearch(0);
  ne.compute(*cloud_normals);
  ROS_DEBUG("Integral image normals took %gms", durationMillis(begin));

  // Remove NaN from pointcloud and normals
  PointIndices::Ptr indices(new PointIndices());
  removeNaNFromPointCloud(*sensor_cloud, *sensor_cloud, indices->indices);
  ExtractIndices<Normal> extract;
  // Extract the inliers
  extract.setInputCloud(cloud_normals);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*cloud_normals);
}

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
bool segmentation::CloudSegmentator::fitPlaneFromNormals(const PointCloudTPtr &input, PointCloudNormalPtr &normals,
                                                         ModelCoefficients::Ptr &coefficients,
                                                         PointIndices::Ptr &inliers)
{
  clock_t begin = clock();
  // Intialize the SACSegmentationFromNormals object
  SACSegmentationFromNormals<PointXYZ, Normal> seg;
  seg.setModelType(SACMODEL_NORMAL_PLANE);
  seg.setMethodType(SAC_RANSAC);

  /************** SACSegmentationFromNormals **************/
  // Set the relative weight (between 0 and 1) to give to the angular distance
  // (0 to pi/2) between point normals and the plane normal.
  seg.setNormalDistanceWeight(cfg.normalDistanceWeightParam);

  // Set the distance we expect a plane model to be from the origin. I hadn't found implementation details.
  seg.setDistanceFromOrigin(cfg.originDistanceParam);

  /******************* SACSegmentation ********************/
  seg.setDistanceThreshold(cfg.distanceThresholdParam);
  seg.setMaxIterations(cfg.maxIterationsParam);
  seg.setOptimizeCoefficients(cfg.optimizeCoefficientsParam);

  // Set the probability of choosing at least one sample free from outliers.
  seg.setProbability(cfg.probabilityParam);

  // Set the maximum distance allowed when drawing random samples.
  PointCloudTPtr temp_cloud(new PointCloudT);
  copyPointCloud(*input, *temp_cloud);

  //SACSegmentation<Normal>::SearchPtr search(new search::KdTree<Normal>);
  SACSegmentation<PointXYZ>::SearchPtr search(new search::KdTree<PointXYZ>);
  search->setInputCloud(temp_cloud);
  seg.setSamplesMaxDist(cfg.sampleMaxDistanceParam, search);

  if (cfg.useSpecificPlaneParam)
  {
    Eigen::Vector3f axis(cfg.planeXParam, cfg.planeYParam, cfg.planeZParam);
    // Set the axis along which we need to search for a model perpendicular to.
    seg.setAxis(axis);
  }

  // Set the angle epsilon (delta) threshold. The maximum allowed difference between the model
  // normal and the given axis in radians. Specify the angle of the normals of the above plane
  seg.setEpsAngle(cfg.epsAngleParam);

  seg.setInputCloud(input);
  seg.setInputNormals(normals);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) return false;

  ROS_DEBUG_STREAM("PLANE FOUND. Model coefficients: " << coefficients->values[0]
                   << " " << coefficients->values[1] << " " << coefficients->values[2] << " " <<
                   coefficients->values[3]);

  ROS_DEBUG_STREAM("Model inliers: " << inliers->indices.size());

  ROS_DEBUG("Plane segmentation took %gms", durationMillis(begin));
  return true;
}

void segmentation::CloudSegmentator::extractPlaneCloud(const PointCloudTPtr &input, PointIndices::Ptr &inliers)
{
  // Create the filtering object
  ExtractIndices<PointXYZ> extract;

  // Extract the inliers
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.filter(*plane_cloud_);
  ROS_DEBUG("Extracted PointCloud representing the planar component");
}

void segmentation::CloudSegmentator::projectOnPlane(const PointCloudTPtr &sensor_cloud,
                                                    const ModelCoefficients::Ptr &table_coefficients,
                                                    const PointIndices::Ptr &tableInliers,
                                                    PointCloudTPtr &projectedTableCloud)
{
  clock_t begin = clock();
  ProjectInliers<PointXYZ> proj;
  proj.setModelType(SACMODEL_PLANE);
  proj.setIndices(tableInliers);
  proj.setInputCloud(sensor_cloud);
  proj.setModelCoefficients(table_coefficients);
  proj.filter(*projectedTableCloud);
  ROS_DEBUG("Project on plane took %gms", durationMillis(begin));
}

void segmentation::CloudSegmentator::computeTableConvexHull(const PointCloudTPtr &projectedTableCloud,
                                                            PointCloudTPtr &tableConvexHull)
{
  clock_t begin = clock();
  ConvexHull<PointXYZ> chull;
  chull.setInputCloud(projectedTableCloud);
  chull.reconstruct(*tableConvexHull);
  tableConvexHull->push_back(tableConvexHull->at(0));
  ROS_DEBUG("Convex hull took %gms", durationMillis(begin));
  ROS_DEBUG("Convex hull has: %lu data points.", tableConvexHull->points.size());
}

bool segmentation::CloudSegmentator::extractCloudOverTheTable(const PointCloudTPtr &sensor_cloud,
                                                              const PointCloudTPtr &tableConvexHull,
                                                              PointCloudTPtr &cloudOverTheTable)
{
  clock_t begin = clock();
  // Segment those points that are in the polygonal prism
  ExtractPolygonalPrismData<PointXYZ> prism;
  // Objects must lie between minHeight and maxHeight m over the plane
  prism.setHeightLimits(cfg.minHeightParam, cfg.maxHeightParam);
  prism.setInputCloud(sensor_cloud);
  prism.setInputPlanarHull(tableConvexHull);
  PointIndices::Ptr indicesOverTheTable(new PointIndices());
  prism.segment(*indicesOverTheTable);

  if (indicesOverTheTable->indices.size() == 0)
  {
    ROS_DEBUG("Extract cloud over the table took %gms", durationMillis(begin));
    ROS_WARN("No points over the table");
    return false;
  }

  // Extraxt indices over the table
  ExtractIndices<PointXYZ> extractIndices;
  extractIndices.setInputCloud(sensor_cloud);
  extractIndices.setIndices(indicesOverTheTable);
  extractIndices.filter(*cloudOverTheTable);
  ROS_DEBUG("Extract cloud over the table took %gms", durationMillis(begin));
  return true;
}


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
void segmentation::CloudSegmentator::clusterObjects(const PointCloudTPtr &cloud_over_table)
{
  clock_t begin = clock();
  // Creating the KdTree object for the search method of the extraction
  search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
  tree->setInputCloud(cloud_over_table);

  EuclideanClusterExtraction<PointXYZ> ec;
  ec.setClusterTolerance(cfg.clusterTolerance);
  ec.setMinClusterSize(cfg.minClusterSize);
  ec.setMaxClusterSize(cfg.maxClusterSize);

  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_over_table);

  std::vector<PointIndices> cluster_indices;
  ec.extract(cluster_indices);

  cloud_cluster_vector_.resize(cluster_indices.size());

  std::stringstream ss;
  for (size_t i = 0; i < cluster_indices.size(); ++i)
  {
    PointCloudTPtr cloudCluster(new PointCloudT);
    PointIndices::Ptr cluster(new PointIndices(cluster_indices[i]));

    // Create the filtering object
    ExtractIndices<PointXYZ> extract;

    // Extract the inliers
    extract.setInputCloud(cloud_over_table);
    extract.setIndices(cluster);
    extract.filter(*cloudCluster);

    cloudCluster->header = sensor_cloud_->header;

    // Store the clusters in a vector. It's the best way?
    cloud_cluster_vector_[i] = cloudCluster;

    ss << cloudCluster->points.size() << ", ";
  }
  ROS_DEBUG_STREAM("Found " << cluster_indices.size() << " clusters: [" << ss << "]");
  ROS_DEBUG("Cluster extraction took %gms", durationMillis(begin));
}

void segmentation::CloudSegmentator::execute()
{
  clock_t beginCallback = clock();

  // If no cloud or no new images since last time, do nothing.
  if (sensor_cloud_->size() == 0 || last_seen_seq_ == sensor_cloud_->header.seq)
  {
    // Clean data
    cloud_cluster_vector_.clear();
    return;
  }

  // Update seq
  last_seen_seq_ = sensor_cloud_->header.seq;

  // Copy the shared pointer to ensure that the cloud won't change in the middle of the processing (
  // if using ASyncSpinner). Shared pointers copies are atomic operations which means that this callback is
  // thread-safe. This is required to convert this node to a nodelet for instance and also to deal with more
  // complex GUI that may require you to use the ASyncSpinner. Note that no mutex are needed here, even in this case.
  PointCloud<pcl::PointXYZ>::Ptr &sensor_cloud = sensor_cloud_;

  // Check if computation succeded
  if (doProcessing(sensor_cloud))
    ROS_INFO("Callback took %gms\n\n", durationMillis(beginCallback));
}

bool segmentation::CloudSegmentator::doProcessing(const PointCloudTPtr &input)
{
  boost::mutex::scoped_lock updateLock(update_normals_mutex_);   // Init smoothed_cloud_ and cloud_normals_ mutex

  // PointCloud cropping
  cropOrganizedPointCloud(input, smoothed_cloud_);
  // Compute integral image normals
  computeNormalsEfficiently(smoothed_cloud_, cloud_normals_);

  bool new_normals = cloud_normals_->size() != 0;
  // Update visualization if there are normals
  point_clouds_updated_ = new_normals;
  updateLock.unlock();                                        // End smoothed_cloud_ and cloud_normals_ mutex

  if (!new_normals)
  {
    ROS_WARN("No normals from cloud");
    return false;
  }


  // Segment plane using normals
  PointIndices::Ptr tableInliers(new PointIndices());
  // Only continue if we find a plane
  if (!fitPlaneFromNormals(smoothed_cloud_, cloud_normals_, table_coefficients_, tableInliers))
    return false;


  if (tableInliers->indices.size() == 0)
  {
    ROS_WARN("No inliers in plane");
    return false;
  }

  extractPlaneCloud(smoothed_cloud_, tableInliers);
  plane_updated_ = true;

  // Project plane points (inliers) in model plane
  PointCloudTPtr projectedTableCloud(new PointCloudT);
  projectOnPlane(smoothed_cloud_, table_coefficients_, tableInliers, projectedTableCloud);
  publish(pub_planar_, projectedTableCloud);

  // Create a Convex Hull representation of the projected inliers
  PointCloudTPtr tableConvexHull(new PointCloudT);
  computeTableConvexHull(projectedTableCloud, tableConvexHull);

  // Extract points over the table's convex hull
  // Only continue if there are points over the table
  if (!extractCloudOverTheTable(smoothed_cloud_, tableConvexHull, cloud_over_table_))
    return false;

  publish(pub_objects_, cloud_over_table_);
  cloud_over_table_updated_ = true;

  // Clustering objects over the table
  clusterObjects(cloud_over_table_);
  //publish(pubObjects, );
  clusters_updated_ = true;
  return true;
}

PointCloudTPtr segmentation::CloudSegmentator::getCluster(size_t index)
{
  return cloud_cluster_vector_[index];
}

const pcl::ModelCoefficientsPtr segmentation::CloudSegmentator::getTable()
{
  return table_coefficients_;
}
} // namespace bachelors_final_project