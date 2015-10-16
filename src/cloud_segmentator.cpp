#include "cloud_segmentator.h"

#include <dynamic_reconfigure/server.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/integral_image_normal.h>

#include <pcl/surface/convex_hull.h>

#include "utils.h"
#include "viewer_spawner.h"

using namespace pcl;

namespace bachelors_final_project
{

/*
 * Constructor
 */
segmentation::CloudSegmentator::CloudSegmentator(ros::NodeHandle nh)
{
  std::cout << "Init cloud constructor!" << std::endl;
  // Create a ROS publisher
  pub_planar_ = nh.advertise<PointCloudT>("planar", 1);
  pub_objects_ = nh.advertise<PointCloudT>("objects", 1);

  // Initialize pointers to point clouds
  sensor_cloud_.reset(new PointCloudT);
  cropped_cloud_.reset(new PointCloudT);
  plane_cloud_.reset(new PointCloudT);
  cloud_normals_.reset(new PointCloud<Normal>);
  table_coefficients_.reset(new ModelCoefficients());
  cloud_over_table_.reset(new PointCloudT);
  cropped_cloud_base_frame.reset(new PointCloudT);

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

void segmentation::CloudSegmentator::cropOrganizedPointCloud(const PointCloudTPtr &cloud_in,
                                                             PointCloudTPtr &cropped_cloud)
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
      if (init_col + u >= cloud_in->width || init_row + v >= cloud_in->height)
      {
        ROS_ERROR("Index out of bounds in cropping. Size is %dx%d and tried to access %dx%d",
                  cloud_in->width, cloud_in->height, init_col + u, init_row + v);
        ros::shutdown();
        exit(-2);
      }
      // We can't use cropped_cloud->push_back because it breaks the organization
      cropped_cloud->at(u, v) = cloud_in->at(init_col + u, init_row + v);
    }
  }

  cropped_cloud->is_dense = cloud_in->is_dense;

  ROS_DEBUG("PointCloud cropping took %gms", durationMillis(begin));
  ROS_DEBUG("Cropped from %lu to %lu", cloud_in->points.size(), cropped_cloud->points.size());
}


void segmentation::CloudSegmentator::computeNormalsEfficiently(const PointCloudTPtr &sensor_cloud,
                                                               PointCloudNormalPtr &cloud_normals)
{
  clock_t begin = clock();
  IntegralImageNormalEstimation<PointT, Normal> ne;

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
  extractPointCloud(cloud_normals, indices, cloud_normals);
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
  SACSegmentationFromNormals<PointT, Normal> seg;
  //seg.setModelType(SACMODEL_NORMAL_PLANE);
  seg.setModelType(SACMODEL_PERPENDICULAR_PLANE);
  //seg.setModelType(SACMODEL_NORMAL_PARALLEL_PLANE);
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
  SACSegmentation<PointT>::SearchPtr search(new search::KdTree<PointT>);
  search->setInputCloud(temp_cloud);
  seg.setSamplesMaxDist(cfg.sampleMaxDistanceParam, search);

  if (cfg.useSpecificPlaneParam) setPlaneAxis(seg);

  seg.setInputCloud(input);
  seg.setInputNormals(normals);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    ROS_WARN("No inliers in plane");
    return false;
  }

  ROS_DEBUG_STREAM("PLANE FOUND. Model coefficients: " <<
                   coefficients->values[0] << " " << coefficients->values[1] << " " <<
                   coefficients->values[2] << " " << coefficients->values[3]);

  ROS_DEBUG("Model inliers: %lu", inliers->indices.size());

  ROS_DEBUG("Plane segmentation took %gms", durationMillis(begin));
  return true;
}

void segmentation::CloudSegmentator::setPlaneAxis(SACSegmentationFromNormals<PointT, Normal> &seg)
{
  PointT normal_base_frame((float) cfg.planeXParam, (float) cfg.planeYParam, (float) cfg.planeZParam);
  if (transformPoint(FOOTPRINT_FRAME, sensor_cloud_->header.frame_id, normal_base_frame, plane_normal_kinect_frame_,
                     sensor_cloud_->header.stamp, tf_listener_))
  {
    plane_normal_base_frame_ = normal_base_frame;

    Eigen::Vector3f axis(plane_normal_kinect_frame_.x, plane_normal_kinect_frame_.y,
                         plane_normal_kinect_frame_.z);
    // Set the axis along which we need to search for a model perpendicular to.
    seg.setAxis(axis);

    // Set the angle epsilon (delta) threshold. The maximum allowed difference between the model
    // normal and the given axis in radians. Specify the angle of the normals of the above plane
    seg.setEpsAngle(cfg.epsAngleParam * M_PI / 180.0);
  }

  transformPointCloud(sensor_cloud_->header.frame_id, FOOTPRINT_FRAME, cropped_cloud_, cropped_cloud_base_frame,
                      sensor_cloud_->header.stamp, tf_listener_);

}

void segmentation::CloudSegmentator::projectOnPlane(const PointCloudTPtr &sensor_cloud,
                                                    const ModelCoefficientsPtr &table_coefficients,
                                                    const PointIndices::Ptr &tableInliers,
                                                    PointCloudTPtr &projectedTableCloud)
{
  clock_t begin = clock();
  ProjectInliers<PointT> proj;
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
  ConvexHull<PointT> chull;
  chull.setInputCloud(projectedTableCloud);
  chull.reconstruct(*tableConvexHull);
  tableConvexHull->push_back(tableConvexHull->at(0));
  ROS_DEBUG("Convex hull took %gms", durationMillis(begin));
  ROS_DEBUG("Convex hull has: %lu data points.", tableConvexHull->points.size());
}

bool segmentation::CloudSegmentator::extractCloudOverTheTable(const PointCloudTPtr &sensor_cloud,
                                                              const PointCloudTPtr &tableConvexHull,
                                                              PointCloudTPtr &cloud_over_table)
{
  clock_t begin = clock();
  // Segment those points that are in the polygonal prism
  ExtractPolygonalPrismData<PointT> prism;
  // Objects must lie between minHeight and maxHeight m over the plane
  prism.setHeightLimits(cfg.minHeightParam, cfg.maxHeightParam);
  prism.setInputCloud(sensor_cloud);
  prism.setInputPlanarHull(tableConvexHull);
  PointIndices::Ptr indices_over_table(new PointIndices());
  prism.segment(*indices_over_table);

  if (indices_over_table->indices.size() == 0)
  {
    ROS_WARN("No points over the table. Took %gms", durationMillis(begin));
    return false;
  }

  // Extraxt indices over the table
  extractPointCloud(sensor_cloud, indices_over_table, cloud_over_table);
  ROS_DEBUG("Extract cloud over the table [%lu points] took %gms",
            indices_over_table->indices.size(), durationMillis(begin));
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
  search::KdTree<PointT>::Ptr tree(new search::KdTree<PointT>);
  tree->setInputCloud(cloud_over_table);

  EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cfg.clusterTolerance);
  ec.setMinClusterSize(cfg.minClusterSize);
  ec.setMaxClusterSize(cfg.maxClusterSize);

  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_over_table);

  std::vector<PointIndices> cluster_indices;
  ec.extract(cluster_indices);
  ROS_DEBUG("Cluster extraction took %gms", durationMillis(begin));

  begin = clock();

  cloud_cluster_vector_.resize(cluster_indices.size());

  std::stringstream ss;
  for (size_t i = 0; i < cluster_indices.size(); ++i)
  {
    PointCloudTPtr cloudCluster(new PointCloudT);
    PointIndices::Ptr cluster(new PointIndices(cluster_indices[i]));

    extractPointCloud(cloud_over_table, cluster, cloudCluster);
    cloudCluster->header = sensor_cloud_->header;

    // Store the clusters in a vector. It's the best way?
    cloud_cluster_vector_[i] = cloudCluster;

    int cluster_size = (int) cloudCluster->points.size();
    ss << cluster_size << ", ";
  }
  ROS_DEBUG_STREAM("Found " << cluster_indices.size() << " clusters: [" << ss.str().c_str() << "]");
  ROS_DEBUG("Cluster construction took %gms", durationMillis(begin));
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
  PointCloudTPtr &sensor_cloud = sensor_cloud_;

  // Check if computation succeded
  if (doProcessing(sensor_cloud))
    ROS_INFO("Callback took %gms\n\n", durationMillis(beginCallback));
  else
    clearSegmentation();
}

bool segmentation::CloudSegmentator::doProcessing(const PointCloudTPtr &input)
{
  ROS_DEBUG("Do Processing!");
  boost::mutex::scoped_lock updateLock(update_normals_mutex_);   // Init cropped_cloud_ and cloud_normals_ mutex

  // PointCloud cropping
  cropOrganizedPointCloud(input, cropped_cloud_);
  // Compute integral image normals
  computeNormalsEfficiently(cropped_cloud_, cloud_normals_);

  bool new_normals = cloud_normals_->size() != 0;
  // Update visualization if there are normals
  point_clouds_updated_ = new_normals;
  updateLock.unlock();                                        // End cropped_cloud_ and cloud_normals_ mutex

  if (!new_normals)
  {
    ROS_WARN("No normals from cloud");
    return false;
  }


  // Segment plane using normals
  PointIndices::Ptr tableInliers(new PointIndices());
  // Only continue if we find a plane
  if (!fitPlaneFromNormals(cropped_cloud_, cloud_normals_, table_coefficients_, tableInliers))
    return false;


  extractPointCloud(cropped_cloud_, tableInliers, plane_cloud_);
  plane_updated_ = true;

  // Project plane points (inliers) in model plane
  PointCloudTPtr projectedTableCloud(new PointCloudT);
  projectOnPlane(cropped_cloud_, table_coefficients_, tableInliers, projectedTableCloud);
  publish(pub_planar_, projectedTableCloud);

  // Create a Convex Hull representation of the projected inliers
  PointCloudTPtr tableConvexHull(new PointCloudT);
  computeTableConvexHull(projectedTableCloud, tableConvexHull);

  // Extract points over the table's convex hull
  // Only continue if there are points over the table
  if (!extractCloudOverTheTable(cropped_cloud_, tableConvexHull, cloud_over_table_))
    return false;

  publish(pub_objects_, cloud_over_table_);
  cloud_over_table_updated_ = true;

  // Clustering objects over the table
  clusterObjects(cloud_over_table_);
  //publish(pubObjects, );
  clusters_updated_ = true;
  return true;
}

void segmentation::CloudSegmentator::clearSegmentation()
{
  plane_cloud_->clear();
  table_coefficients_->values.clear();
  cloud_over_table_->clear();
  cloud_cluster_vector_.clear();
}

PointCloudTPtr segmentation::CloudSegmentator::getCluster(size_t index)
{
  return cloud_cluster_vector_[index];
}

const ModelCoefficientsPtr segmentation::CloudSegmentator::getTable()
{
  return table_coefficients_;
}
} // namespace bachelors_final_project