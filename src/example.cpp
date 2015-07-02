#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/features/integral_image_normal.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <dynamic_reconfigure/server.h>
#include <bachelors_final_project/ParametersConfig.h>

using namespace pcl;
using namespace std;

ros::Publisher pubDownsampled, pubPlanar, pubObjects;
PCLPointCloud2 pointCloudMsg;

float normalDistanceWeight;
float distanceThreshold;
float maxDepthChangeFactor;
float normalSmoothingSize;
//  Leaf size of 0.5cm
float leafSize;
float minHeight;
float maxHeight;
bool useDepthDependentSmoothing;
bool defaultParams;

void resetParams(){
  normalDistanceWeight = 0.1;
  distanceThreshold = 0.1;
  maxDepthChangeFactor = 0.02f;
  normalSmoothingSize = 10.0f;
  leafSize = 0.005f;
  minHeight = 0.0;
  maxHeight = 0.2;
  useDepthDependentSmoothing = true;
  defaultParams = false;
}

void parameterCallback(bachelors_final_project::ParametersConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %g %g %g %g %g %g %s",
           config.normalDistanceWeight,
           config.distanceThreshold,
           config.normalSmoothingSize,
           config.leafSize,
           config.minHeight,
           config.maxHeight,
           config.useDepthDependentSmoothing?"True":"False",
           config.defaultParams?"True":"False");

  if (config.defaultParams){
    config.normalDistanceWeight = 0.1;
    config.distanceThreshold = 0.1;
    config.maxDepthChangeFactor = 0.02f;
    config.normalSmoothingSize = 10.0f;
    config.leafSize = 0.005f;
    config.minHeight = 0.0;
    config.maxHeight = 0.2;
    config.useDepthDependentSmoothing = true;
    config.defaultParams = false;
  }

  normalDistanceWeight = config.normalDistanceWeight;
  distanceThreshold = config.distanceThreshold;
  maxDepthChangeFactor = config.maxDepthChangeFactor;
  normalSmoothingSize = config.normalSmoothingSize;
  leafSize = config.leafSize;
  minHeight = config.minHeight;
  maxHeight = config.maxHeight;
  useDepthDependentSmoothing = config.useDepthDependentSmoothing;
  defaultParams = config.defaultParams;
}

/* Use SACSegmentation to find the dominant plane in the scene
 * Inputs:
 *   input
 *     The input point cloud
 *   max_iterations
 *     The maximum number of RANSAC iterations to run
 *   distanceThreshold
 *     The inlier/outlier threshold.  Points within this distance
 *     from the hypothesized plane are scored as inliers.
 * Return: A pointer to the ModelCoefficients (i.e., the 4 coefficients of the plane,
 *         represented in c0*x + c1*y + c2*z + c3 = 0 form)
 */
bool
fitPlaneFromNormals (const PointCloud<PointXYZ>::Ptr &input, PointCloud<Normal>::Ptr &normals, float max_iterations,
                     ModelCoefficients::Ptr &coefficients, PointIndices::Ptr &inliers)
{
  // Intialize the SACSegmentationFromNormals object
  SACSegmentationFromNormals<PointXYZ, Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_NORMAL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (max_iterations);
  // Set the relative weight (between 0 and 1) to give to the angular distance
  // (0 to pi/2) between point normals and the plane normal.
  seg.setNormalDistanceWeight (normalDistanceWeight);
  seg.setDistanceThreshold (distanceThreshold);

  // here specify the plane i.e X,Y,Z
  //Eigen::Vector3f axis = Eigen::Vector3f(0.0,1.0,0.0);
  // Set the axis along which we need to search for a model perpendicular to.
  //seg.setAxis(axis);
  // Set the angle epsilon (delta) threshold. The maximum allowed difference between the model
  // normal and the given axis in radians. Specify the angle of the normals of the above plane
  //seg.setEpsAngle(20*M_PI/180);
  // Set the distance we expect a plane model to be from the origin.
  //seg.setDistanceFromOrigin(2.0);
  
  seg.setInputCloud (input);
  
  seg.setInputNormals (normals);

  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    ROS_WARN ("Could not estimate a planar model for the given dataset.");
    return false;
  }

  ROS_DEBUG_STREAM("Model coefficients: " << coefficients->values[0] << " "
                                                                     << coefficients->values[1] << " "
                                                                     << coefficients->values[2] << " "
                                                                     << coefficients->values[3]);

  ROS_DEBUG_STREAM("Model inliers: " << inliers->indices.size ());
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    ROS_DEBUG_STREAM(inliers->indices[i] << "    "
                     << input->points[inliers->indices[i]].x << " "
                                                             << input->points[inliers->indices[i]].y << " "
                                                             << input->points[inliers->indices[i]].z);
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
bool
clusterObjects (const PointCloud<PointXYZ>::Ptr & cloudOverTheTable, float clusterTolerance,
                int minClusterSize, int maxClusterSize)
{  
  // Creating the KdTree object for the search method of the extraction
  search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
  tree->setInputCloud (cloudOverTheTable);

  EuclideanClusterExtraction<PointXYZ> ec;
  ec.setClusterTolerance (clusterTolerance);
  ec.setMinClusterSize (minClusterSize);
  ec.setMaxClusterSize (maxClusterSize);

  ec.setSearchMethod (tree);
  ec.setInputCloud (cloudOverTheTable);
  
  vector<PointIndices> clusterIndices;

  ec.extract (clusterIndices);

  vector<PointCloud<PointXYZ>::Ptr > cloudClusterVector;

  int j = 0;
  for (vector<PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
  {
    PointCloud<PointXYZ>::Ptr cloudCluster (new PointCloud<PointXYZ>);

    for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloudCluster->points.push_back (cloudOverTheTable->points[*pit]);
    
    cloudCluster->width = cloudCluster->points.size ();
    cloudCluster->height = 1;
    cloudCluster->is_dense = true;

    // Store the clusters in a vector. It's the best way?
    cloudClusterVector.push_back (cloudCluster);

    ROS_DEBUG_STREAM("PointCloud representing the Cluster: " << cloudCluster->points.size () << " data points.");
    stringstream ss;
    ss << "cloudCluster_" << j << ".pcd";
    //writer.write<PointXYZ> (ss.str (), *cloudCluster, false);
    j++;
  }
}

void
computeNormalsEfficiently(const PointCloud<PointXYZ>::Ptr &sensorCloud, int normalEstimationMethod,
                          PointCloud<Normal>::Ptr &cloudNormals)
{  
  IntegralImageNormalEstimation<PointXYZ, Normal> ne;

  switch (normalEstimationMethod)
  {
    case 1:
      ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
      break;
    case 2:
      ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
      break;
    case 3:
      ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
      break;
    case 4:
      ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
      break;
    default:
      ROS_ERROR("Wrong Normal estimation method. Parameter error");
      ros::shutdown();
      return;
  }
  
  ne.setMaxDepthChangeFactor(maxDepthChangeFactor);
  ne.setDepthDependentSmoothing(useDepthDependentSmoothing);
  ne.setNormalSmoothingSize(normalSmoothingSize);
  ne.setInputCloud(sensorCloud);
  ne.compute(*cloudNormals);
}

void
projectOnPlane(const PointCloud<PointXYZ>::Ptr &sensorCloud, const ModelCoefficients::Ptr &tableCoefficients,
               const PointIndices::Ptr &tableInliers, PointCloud<PointXYZ>::Ptr &projectedTableCloud)
{
  ProjectInliers<PointXYZ> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setIndices (tableInliers);
  proj.setInputCloud (sensorCloud);
  proj.setModelCoefficients (tableCoefficients);
  proj.filter (*projectedTableCloud);
}

void
computeTableConvexHull(const PointCloud<PointXYZ>::Ptr& projectedTableCloud, PointCloud<PointXYZ>::Ptr& tableConvexHull)
{
  ConvexHull<PointXYZ> chull;
  chull.setInputCloud (projectedTableCloud);
  chull.reconstruct (*tableConvexHull);
  ROS_DEBUG_STREAM("Convex hull has: " << tableConvexHull->points.size () << " data points.");
}

bool
extractCloudOverTheTable(const PointCloud<PointXYZ>::Ptr& sensorCloud,
                         const PointCloud<PointXYZ>::Ptr& tableConvexHull,
                         PointCloud<PointXYZ>::Ptr& cloudOverTheTable)
{
  // Segment those points that are in the polygonal prism
  ExtractPolygonalPrismData<PointXYZ> prism;
  // Objects must lie between minHeight and maxHeight m over the plane
  prism.setHeightLimits (minHeight, maxHeight);
  prism.setInputCloud (sensorCloud);
  prism.setInputPlanarHull (tableConvexHull);
  PointIndices::Ptr indicesOverTheTable (new PointIndices ());
  prism.segment (*indicesOverTheTable);

  if(indicesOverTheTable->indices.size () == 0)
  {
    ROS_WARN("No points over the table");
    return false;
  }

  // Extraxt indices over the table
  ExtractIndices<PointXYZ> extractIndices;
  extractIndices.setInputCloud(sensorCloud);
  extractIndices.setIndices(indicesOverTheTable);
  extractIndices.filter(*cloudOverTheTable);
  return true;
}

void
downsample(const PointCloud<PointXYZ>::Ptr& input, PointCloud<PointXYZ>::Ptr& output)
{
  // Create the filtering object: downsample the dataset
  pcl::VoxelGrid<PointXYZ> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (leafSize, leafSize, leafSize);
  sor.filter (*output);
}

void
publish(const ros::Publisher pub, const PointCloud<PointXYZ>::Ptr cloud)
{
  toPCLPointCloud2 (*cloud, pointCloudMsg);
  pub.publish(pointCloudMsg);
}

void 
sensorCallback (const PCLPointCloud2::ConstPtr& sensorInput)
{
  ROS_INFO_ONCE("Callback Called");
  PointCloud<PointXYZ>::Ptr sensorCloud(new PointCloud<PointXYZ>);
  fromPCLPointCloud2 (*sensorInput,*sensorCloud);

  //  pcl::PassThrough<PointT> pass;
  //  // Build a passthrough filter to remove spurious NaNs
  //  pass.setInputCloud (cloud);
  //  pass.setFilterFieldName ("z");
  //  pass.setFilterLimits (0, 1.5);
  //  pass.filter (*cloud_filtered);
  //  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // Downsample
  PointCloud<PointXYZ>::Ptr downsampledCloud (new PointCloud<PointXYZ>);
  downsample(sensorCloud, downsampledCloud);
  publish(pubDownsampled, downsampledCloud);

  // Compute integral image normals
  PointCloud<Normal>::Ptr cloudNormals (new PointCloud<Normal>);
  computeNormalsEfficiently(sensorCloud, 2, cloudNormals);
  
  // Segment plane using normals
  ModelCoefficients::Ptr tableCoefficients (new ModelCoefficients ());
  PointIndices::Ptr tableInliers (new PointIndices ());
  // Only continue if we find a plane
  if (fitPlaneFromNormals(sensorCloud, cloudNormals, 5000, tableCoefficients, tableInliers))
    return;

  // Project plane points (inliers) in model plane
  PointCloud<PointXYZ>::Ptr projectedTableCloud (new PointCloud<PointXYZ>);
  projectOnPlane(sensorCloud, tableCoefficients, tableInliers, projectedTableCloud);
  publish(pubPlanar, projectedTableCloud);

  // Create a Convex Hull representation of the projected inliers
  PointCloud<PointXYZ>::Ptr tableConvexHull(new PointCloud<PointXYZ>);
  computeTableConvexHull(projectedTableCloud, tableConvexHull);

  // Extract points over the table's convex hull
  PointCloud<PointXYZ>::Ptr cloudOverTheTable(new PointCloud<PointXYZ>);
  // Only continue if there are points over the table
  if(extractCloudOverTheTable(sensorCloud, tableConvexHull, cloudOverTheTable))
    return;
  publish(pubObjects, cloudOverTheTable);

  // Clustering objects over the table
  //clusterObjects(cloudOverTheTable, 0.02, 100, 25000);
  //publish(pubObjects, );
}

int
main (int argc, char** argv)
{
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("DEBUG ACTIVATED");

  // Initialize ROS
  ros::init (argc, argv, "bachelors_final_project");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, sensorCallback);

  // Create a ROS publisher
  pubDownsampled = nh.advertise<PCLPointCloud2> ("downsampled", 1);
  pubPlanar = nh.advertise<PCLPointCloud2> ("planar", 1);
  pubObjects = nh.advertise<PCLPointCloud2> ("objects", 1);

  resetParams();

  dynamic_reconfigure::Server<bachelors_final_project::ParametersConfig> server;
  dynamic_reconfigure::Server<bachelors_final_project::ParametersConfig>::CallbackType f;

  f = boost::bind(&parameterCallback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Escuchando");

  // Spin
  ros::spin ();
}
