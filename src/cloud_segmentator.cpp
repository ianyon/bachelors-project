#include "cloud_segmentator.h"

#include <pcl_ros/transforms.h>

#include <dynamic_reconfigure/server.h>

#include <pcl/common/angles.h>
#include <pcl/filters/convolution_3d.h>

#include "utils.h"

using namespace pcl;

namespace bachelors_final_project
{


/*
 * Constructor
 */
segmentation::CloudSegmentator::CloudSegmentator(ros::NodeHandle nh, tf::TransformListener &tf_listener) :
    search_(new search::OrganizedNeighbor<Point>),
    processed_cloud_(false),
    plane_updated_(false),
    point_clouds_updated_(false),
    cloud_over_table_updated_(false),
    clusters_updated_(false),
    last_seen_seq_(0),
    pub_planar_(nh.advertise<Cloud>("table", 1)),
    pub_over_table_(nh.advertise<Cloud>("objects_over_table", 1)),
    pub_noisy_(nh.advertise<Cloud>("noisy", 1)),
    // Initialize pointers to point clouds
    sensor_cloud_(new Cloud),
    cropped_cloud_(new Cloud),
    plane_cloud_(new Cloud),
    cloud_normals_(new CloudNormal),
    table_coefficients_(new ModelCoefficients),
    cloud_over_table_(new Cloud),
    cropped_cloud_base_frame(new Cloud),
    indices_over_table_(new PointIndices),
    tf_listener_(tf_listener),
    projected_table_cloud_(new Cloud),
    nor_dis(rng, boost::normal_distribution<>(0.0, 0.0015)),
    nor_dis_low(rng, boost::normal_distribution<>(0.0, 0.001))
{
  sac_segmentation_.setModelType(SACMODEL_NORMAL_PARALLEL_PLANE);
  sac_segmentation_.setMethodType(SAC_RANSAC);
}  // end CloudSegmentator()

void segmentation::CloudSegmentator::updateConfig(ParametersConfig &config)
{
  cfg = config;
}

void segmentation::CloudSegmentator::sensorCallback(const CloudConstPtr &sensorInput)
{
  ROS_INFO_ONCE_NAMED(SEGMENTATION(), "Callback Called");
  sensor_cloud_ = sensorInput->makeShared();
}

void segmentation::CloudSegmentator::cropOrganizedPointCloud(const CloudPtr &cloud_in,
                                                             CloudPtr &cropped_cloud)
{
  clock_t begin = clock();
  // Kinect is 640/480
  int width = (int) floor(cfg.scaleParam * cloud_in->width / 10.0),   //64
      height = (int) floor(cfg.scaleParam * cloud_in->height / 10.0); //48
  PointXY in_cloud_mid_point = {cloud_in->width / 2, cloud_in->height / 2};
  // Constrain the init index to a minimum of zero to avoid index out of bounds
  PointXY out_cloud_init_point = {
      (float) fmax(floor(in_cloud_mid_point.x - width / 2 + cfg.xTranslateParam), 0),
      (float) fmax(floor(in_cloud_mid_point.y - height / 2 + cfg.yTranslateParam), 0)};
  // Constrain again the init index to a maximum that ensures that the last index is inside the cloud
  int init_col = (int) fmin(out_cloud_init_point.x, cloud_in->width - width),
      init_row = (int) fmin(out_cloud_init_point.y, cloud_in->height - height);

  setProperties(cloud_in, cropped_cloud, width, height);

  for (unsigned int u = 0; u < width; u++)
  {
    for (unsigned int v = 0; v < height; v++)
    {
      /*ROS_ASSERT_MSG_NAMED(SEGMENTATION(),init_col + u >= cloud_in->width || init_row + v >= cloud_in->height,
                     "Index out of bounds in cropping. Size is %dx%d and tried to access %dx%d",
                     cloud_in->width, cloud_in->height, init_col + u, init_row + v);*/
      // We can't use cropped_cloud->push_back because it breaks the organization
      cropped_cloud->at(u, v) = cloud_in->at(init_col + u, init_row + v);
    }
  }

  cropped_cloud->is_dense = cloud_in->is_dense;

  ROS_INFO_NAMED(SEGMENTATION(), "[%g ms] PointCloud cropping [%lu to %lu]", durationMillis(begin),
                  cloud_in->points.size(), cropped_cloud->points.size());
}


bool segmentation::CloudSegmentator::computeNormalsEfficiently(const CloudPtr &sensor_cloud,
                                                               CloudNormalPtr &cloud_normals)
{
  clock_t begin = clock();
  switch (cfg.normalEstimationMethodParam)
  {
    case 1:
      normal_estimation_.setNormalEstimationMethod(normal_estimation_.COVARIANCE_MATRIX);
      break;
    case 2:
      normal_estimation_.setNormalEstimationMethod(normal_estimation_.AVERAGE_3D_GRADIENT);
      break;
    case 3:
      normal_estimation_.setNormalEstimationMethod(normal_estimation_.AVERAGE_DEPTH_CHANGE);
      break;
    case 4:
      normal_estimation_.setNormalEstimationMethod(normal_estimation_.SIMPLE_3D_GRADIENT);
      break;
    default:
      ROS_ERROR_NAMED(SEGMENTATION(), "Wrong Normal estimation method. Parameter error");
      ros::shutdown();
      exit(-1);
  }

  // First convert from [m] to [cm] then counter the 2 factor:
  // https://github.com/PointCloudLibrary/pcl/blob/master/features/include/pcl/features/impl/integral_image_normal.hpp#L744
  normal_estimation_.setMaxDepthChangeFactor((float) ((cfg.maxDepthChangeFactorParam / 100) / 2.0));
  normal_estimation_.setDepthDependentSmoothing(cfg.useDepthDependentSmoothingParam);
  normal_estimation_.setNormalSmoothingSize((float) cfg.normalSmoothingSizeParam);
  normal_estimation_.setInputCloud(sensor_cloud);
  //ne.setKSearch(0);
  normal_estimation_.compute(*cloud_normals);

  // Remove NaN from pointcloud and normals
  //PointIndices::Ptr indices(new PointIndices());
  // Sometimes the cloud is not dense. In that case removeNaNFromPointCloud'll break the organization
  //ROS_ERROR_COND_NAMED(!sensor_cloud->is_dense, SEGMENTATION(), "Cropped cloud isn't dense. Organization will be lost");
  //removeNaNFromPointCloud(*sensor_cloud, *sensor_cloud, indices->indices);
  //normalPointCloudFromIndices(cloud_normals, indices, cloud_normals);
  ROS_INFO_NAMED(SEGMENTATION(), "[%g ms] Integral image normals", durationMillis(begin));

  return cloud_normals_->size() != 0;
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
bool segmentation::CloudSegmentator::fitPlaneFromNormals(const CloudPtr &input, CloudNormalPtr &normals,
                                                         ModelCoefficients::Ptr &coefficients,
                                                         PointIndices::Ptr &inliers)
{
  clock_t begin = clock();
  /************** SACSegmentationFromNormals **************/
  // Set the relative weight (between 0 and 1) to give to the angular distance
  // (0 to pi/2) between point normals and the plane normal.
  sac_segmentation_.setNormalDistanceWeight(cfg.normalDistanceWeightParam);

  /******************* SACSegmentation ********************/
  sac_segmentation_.setDistanceThreshold(cfg.distanceThresholdParam);
  sac_segmentation_.setMaxIterations(cfg.maxIterationsParam);
  sac_segmentation_.setOptimizeCoefficients(cfg.optimizeCoefficientsParam);

  // Set the probability of choosing at least one sample free from outliers.
  sac_segmentation_.setProbability(cfg.probabilityParam);

  // Set the maximum distance allowed when drawing random samples.
  search_->setInputCloud(input);
  sac_segmentation_.setSamplesMaxDist(cfg.sampleMaxDistanceParam, search_);

  if (cfg.useSpecificPlaneParam) setPlaneAxis(sac_segmentation_);

  sac_segmentation_.setInputCloud(input);
  sac_segmentation_.setInputNormals(normals);
  sac_segmentation_.segment(*inliers, *coefficients);

  bool enough_inliers = inliers->indices.size() > 100; // Sometimes there are few inliers and that's bad
  ROS_WARN_COND_NAMED(!enough_inliers, SEGMENTATION(), "No inliers in plane");
  ROS_INFO_COND_NAMED(enough_inliers, SEGMENTATION(), "[%g ms] Plane segmentation (%lu): [%g %g %g %g]",
                       durationMillis(begin), inliers->indices.size(),
                       coefficients->values[0], coefficients->values[1], coefficients->values[2],
                       coefficients->values[3]);
  return enough_inliers;
}

void segmentation::CloudSegmentator::setPlaneAxis(SACSegmentationFromNormals<Point, Normal> &seg)
{
  //tf::Vector3 normal_base_frame(cfg.planeXParam, cfg.planeYParam, cfg.planeZParam);
  Vec3f normal_base_frame(cfg.planeXParam, cfg.planeYParam, cfg.planeZParam);
  if (transformPoint(FOOTPRINT_FRAME, sensor_cloud_->header.frame_id, normal_base_frame, plane_normal_kinect_frame_,
                     0/*sensor_cloud_->header.stamp*/, tf_listener_))
  {
    plane_normal_base_frame_ = Point((float) normal_base_frame.x(), (float) normal_base_frame.y(),
                                     (float) normal_base_frame.z());

    Vec3f axis(plane_normal_kinect_frame_.x, plane_normal_kinect_frame_.y,
               plane_normal_kinect_frame_.z);
    // Set the axis along which we need to search for a model perpendicular to.
    seg.setAxis(axis);

    // Set the angle epsilon (delta) threshold. The maximum allowed difference between the model
    // normal and the given axis in radians. Specify the angle of the normals of the above plane
    seg.setEpsAngle(pcl::deg2rad(cfg.epsAngleParam));
  }
  else
    throw ComputeFailedException("Without transform can't find horizontal plane to segment table");
}

void segmentation::CloudSegmentator::computeTableConvexHull(const CloudPtr &projected_table_cloud,
                                                            CloudPtr &table_convex_hull)
{
  clock_t begin = clock();
  convex_hull_.setInputCloud(projected_table_cloud);
  convex_hull_.reconstruct(*table_convex_hull);
  table_convex_hull->push_back(table_convex_hull->at(0));
  ROS_INFO_NAMED(SEGMENTATION(), "[%g ms] Convex hull [%lu points]", durationMillis(begin),
                  table_convex_hull->points.size());
}

bool segmentation::CloudSegmentator::extractCloudOverTheTable(const CloudPtr &sensor_cloud,
                                                              const CloudPtr &table_convex_hull,
                                                              PointIndicesPtr &indices_over_table)
{
  clock_t begin = clock();
  // Segment those points that are in the polygonal prism
  // Objects must lie between minHeight and maxHeight m over the plane
  extract_prism_.setHeightLimits(cfg.minHeightParam / 100, cfg.maxHeightParam / 100);
  extract_prism_.setInputCloud(sensor_cloud);
  extract_prism_.setInputPlanarHull(table_convex_hull);
  extract_prism_.segment(*indices_over_table);

  bool empty = indices_over_table->indices.size() == 0;
  ROS_WARN_COND_NAMED(empty, SEGMENTATION(), "No points over the table. Took %g ms", durationMillis(begin));
  ROS_INFO_COND_NAMED(!empty, SEGMENTATION(), "[%g ms] Extract cloud over the table [%lu points]",
                       durationMillis(begin),
                       indices_over_table->indices.size());
  return !empty;
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
bool segmentation::CloudSegmentator::clusterObjects(const CloudPtr &cloud_in,
                                                    const PointIndicesPtr &over_table)
{
  clock_t begin = clock();
  euclidean_clustering_.setClusterTolerance(cfg.clusterTolerance / 100);
  euclidean_clustering_.setMinClusterSize(cfg.minClusterSize);
  euclidean_clustering_.setMaxClusterSize(cfg.maxClusterSize);
  euclidean_clustering_.setInputCloud(cloud_in);
  euclidean_clustering_.setIndices(over_table);
  std::vector<PointIndices> cluster_indices;
  euclidean_clustering_.extract(cluster_indices);

  bool empty = cluster_indices.size() == 0;
  ROS_INFO_COND_NAMED(empty, SEGMENTATION(), "[%g ms] Clustering Failed", durationMillis(begin));

  if (!empty) clusters_vector_.clear();
  std::stringstream ss;
  BOOST_FOREACH(PointIndices indices, cluster_indices)
        {
          CloudPtr cloud_cluster(new Cloud);
          PointIndicesPtr cluster(boost::make_shared<PointIndices>(indices));
          pointCloudFromIndices(cloud_in, cluster, cloud_cluster);
          cloud_cluster->header = sensor_cloud_->header;
          clusters_vector_.push_back(cloud_cluster);
          ss << cloud_cluster->size() << ", ";
        }
  ROS_INFO_COND_NAMED(!empty, SEGMENTATION(), "[%g ms] Clustering [%lu]: [%s]", durationMillis(begin),
                       cluster_indices.size(),
                       ss.str().c_str());
  return !empty;
}

void segmentation::CloudSegmentator::execute()
{
  clock_t beginCallback = clock();

  // If no cloud or no new images since last time, do nothing.
  if (sensor_cloud_->size() == 0 || last_seen_seq_ == sensor_cloud_->header.seq)
  {
    setProcessedCloud(false);
    return;
  }

  // Update seq
  last_seen_seq_ = sensor_cloud_->header.seq;

  // Copy the shared pointer to ensure that the cloud won't change in the middle of the processing (
  // if using ASyncSpinner). Shared pointers copies are atomic operations which means that this callback is
  // thread-safe. This is required to convert this node to a nodelet for instance and also to deal with more
  // complex GUI that may require you to use the ASyncSpinner. Note that no mutex are needed here, even in this case.
  CloudPtr &sensor_cloud = sensor_cloud_;

  // Check if computation updated the clusters
  if (setProcessedCloud(doProcessing(sensor_cloud)))
    ROS_INFO_NAMED(SEGMENTATION(), "[%g ms] Segmentation Success", durationMillis(beginCallback));
  else
    ROS_ERROR_NAMED(SEGMENTATION(), "[%g ms] Segmentation Failed", durationMillis(beginCallback));
}

bool segmentation::CloudSegmentator::doProcessing(const CloudPtr &input)
{
  ROS_INFO_NAMED(SEGMENTATION(), "Processing segmentation!");

  /* ENABLE THIS TO ADD NOISE TO THE KINECT POINTCLOUD
   * for (size_t i = 0; i < input->size(); i++)
  {
    input->points[i].z += nor_dis();
    input->points[i].x += nor_dis_low();
    input->points[i].y += nor_dis_low();
  }
  pub_noisy_.publish(input);*/

  boost::mutex::scoped_lock updateLock(update_normals_mutex_);
  cropOrganizedPointCloud(input, cropped_cloud_);
  bool new_normals = computeNormalsEfficiently(cropped_cloud_, cloud_normals_);
  // Update visualization if there are normals
  point_clouds_updated_ = new_normals;
  updateLock.unlock();
  if (!new_normals)
    return clearSegmentation(NORMALS);

  // Segment plane using normals
  PointIndices::Ptr tableInliers(new PointIndices());
  try
  {
    if (!fitPlaneFromNormals(cropped_cloud_, cloud_normals_, table_coefficients_, tableInliers))
      return clearSegmentation(PLANE);
  }
  catch (ComputeFailedException ex)
  {
    return clearSegmentation(PLANE);
  }
  pointCloudFromIndices(cropped_cloud_, tableInliers, plane_cloud_);
  plane_updated_ = true;

  // Project plane points (inliers) in model plane
  projectOnPlane(cropped_cloud_, table_coefficients_, tableInliers, projected_table_cloud_);
  ROS_INFO_NAMED(SEGMENTATION(), "Project on plane: %lu", projected_table_cloud_->size());
  pub_planar_.publish(projected_table_cloud_);

  // Create a Convex Hull representation of the projected inliers
  CloudPtr table_convex_hull(new Cloud);
  computeTableConvexHull(projected_table_cloud_, table_convex_hull);

  // Extract points over the table's convex hull
  if (!extractCloudOverTheTable(cropped_cloud_, table_convex_hull, indices_over_table_))
    return clearSegmentation(OVER_TABLE);
  pointCloudFromIndices(cropped_cloud_, indices_over_table_, cloud_over_table_);
  pub_over_table_.publish(cloud_over_table_);
  cloud_over_table_updated_ = true;

  // Clustering objects over the table
  if (!clusterObjects(cropped_cloud_, indices_over_table_))
    return clearSegmentation(CLUSTERS);
  clusters_updated_ = true;
  return true;
}

bool segmentation::CloudSegmentator::clearSegmentation(FailedLevel failed_level)
{
  std::stringstream ss;
  switch (failed_level)
  {
    case NORMALS:
      ss << " NORMALS,";
      cloud_normals_->clear();
    case PLANE:
      ss << " PLANE,";
      plane_cloud_->clear();
      table_coefficients_->values.clear();
    case OVER_TABLE:
      ss << " OVER TABLE,";
      cloud_over_table_->clear();
      indices_over_table_->indices.clear();
    case CLUSTERS:
      ss << " CLUSTERS";
      clusters_vector_.clear();
  };
  ROS_ERROR_NAMED(SEGMENTATION(), "Failed in: %s", ss.str().c_str());
  return false;
}

CloudPtr segmentation::CloudSegmentator::getCluster(size_t index)
{
  return clusters_vector_[index];
}

const ModelCoefficientsPtr segmentation::CloudSegmentator::getTable()
{
  return table_coefficients_;
}

const CloudPtr segmentation::CloudSegmentator::getTableCloud()
{
  return projected_table_cloud_;
}

bool segmentation::CloudSegmentator::setProcessedCloud(bool state)
{
  processed_cloud_ = state;
  return processed_cloud_;
}

bool segmentation::CloudSegmentator::noNewProcessedData()
{
  return !processed_cloud_;
}

void segmentation::CloudSegmentator::pointCloudFromIndices(const CloudPtr &input, PointIndicesPtr &inliers,
                                                           CloudPtr &output)
{
  extract_.setInputCloud(input);
  extract_.setIndices(inliers);
  extract_.filter(*output);
}

void segmentation::CloudSegmentator::normalPointCloudFromIndices(const CloudNormalPtr &input,
                                                                 PointIndicesPtr &inliers,
                                                                 CloudNormalPtr &output)
{
  extract_normals_.setInputCloud(input);
  extract_normals_.setIndices(inliers);
  extract_normals_.filter(*output);
}
} // namespace bachelors_final_project