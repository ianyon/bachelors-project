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

using namespace pcl;
using namespace std;

ros::Publisher pubDownsampled, pubPlanar, pubObjects;

/* Use SACSegmentation to find the dominant plane in the scene
 * Inputs:
 *   input 
 *     The input point cloud
 *   max_iterations 
 *     The maximum number of RANSAC iterations to run
 *   distance_threshold 
 *     The inlier/outlier threshold.  Points within this distance
 *     from the hypothesized plane are scored as inliers.
 * Return: A pointer to the ModelCoefficients (i.e., the 4 coefficients of the plane, 
 *         represented in c0*x + c1*y + c2*z + c3 = 0 form)
 */
bool
fitPlaneFromNormals (const PointCloud<PointXYZRGB>::Ptr &input, PointCloud<Normal>::Ptr &normals,
                     float distance_threshold, float max_iterations,
                     ModelCoefficients::Ptr &coefficients, PointIndices::Ptr &inliers)
{
  // Intialize the SACSegmentationFromNormals object  
  SACSegmentationFromNormals<PointXYZRGB, Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (SACMODEL_NORMAL_PLANE);  
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (max_iterations);
  // Set the relative weight (between 0 and 1) to give to the angular distance 
  // (0 to pi/2) between point normals and the plane normal. 
  seg.setNormalDistanceWeight (0.1);
  seg.setDistanceThreshold (distance_threshold);

  // Set the angle epsilon (delta) threshold. The maximum allowed difference between the model
  // normal and the given axis in radians. 
  //seg.setEpsAngle (10*PI/180);
  
  // Set the distance we expect a plane model to be from the origin.
  //seg.setDistanceFromOrigin(2.0);

  // Set the axis along which we need to search for a model perpendicular to. 
  //setAxis (const Eigen::Vector3f &   ax);
  
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
clusterObjects (const PointCloud<PointXYZRGB>::Ptr & cloudOverTheTable, float clusterTolerance,
                int minClusterSize, int maxClusterSize)
{  
  // Creating the KdTree object for the search method of the extraction
  search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
  tree->setInputCloud (cloudOverTheTable);

  EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance (clusterTolerance);
  ec.setMinClusterSize (minClusterSize);
  ec.setMaxClusterSize (maxClusterSize);

  ec.setSearchMethod (tree);
  ec.setInputCloud (cloudOverTheTable);
  
  vector<PointIndices> clusterIndices;

  ec.extract (clusterIndices);

  vector<PointCloud<PointXYZRGB>::Ptr > cloudClusterVector;

  int j = 0;
  for (vector<PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
  {
    PointCloud<PointXYZRGB>::Ptr cloudCluster (new PointCloud<PointXYZRGB>);

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
    //writer.write<PointXYZRGB> (ss.str (), *cloudCluster, false);
    j++;
  }
}

void
computeNormalsEfficiently(const PointCloud<PointXYZRGB>::Ptr &sensorCloud, int normalEstimationMethod,
                          float maxDepthChangeFactor, float normalSmoothingSize, PointCloud<Normal>::Ptr &cloudNormals)
{  
  IntegralImageNormalEstimation<PointXYZRGB, Normal> ne;

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
  ne.setNormalSmoothingSize(normalSmoothingSize);
  ne.setInputCloud(sensorCloud);
  ne.compute(*cloudNormals);
}

bool
projectOnPlane(const PointCloud<PointXYZRGB>::Ptr &sensorCloud, const ModelCoefficients::Ptr &tableCoefficients,
               const PointIndices::Ptr &tableInliers, PointCloud<PointXYZRGB>::Ptr &projectedTableCloud)
{
  ProjectInliers<PointXYZRGB> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setIndices (tableInliers);
  proj.setInputCloud (sensorCloud);
  proj.setModelCoefficients (tableCoefficients);
  proj.filter (*projectedTableCloud);
}

bool
computeTableConvexHull(const PointCloud<PointXYZRGB>::Ptr& projectedTableCloud, PointCloud<PointXYZRGB>::Ptr& tableConvexHull)
{
  ConvexHull<PointXYZRGB> chull;
  chull.setInputCloud (projectedTableCloud);
  chull.reconstruct (*tableConvexHull);
  ROS_DEBUG_STREAM("Convex hull has: " << tableConvexHull->points.size () << " data points.");
}

bool
extractCloudOverTheTable(const PointCloud<PointXYZRGB>::Ptr& sensorCloud, 
                         const PointCloud<PointXYZRGB>::Ptr& tableConvexHull,
                         float minHeight, float maxHeight,
                         PointCloud<PointXYZRGB>::Ptr& cloudOverTheTable)
{
  // Segment those points that are in the polygonal prism
  ExtractPolygonalPrismData<PointXYZRGB> prism;
  // Objects must lie between minHeight and maxHeight m over the plane
  prism.setHeightLimits (minHeight, maxHeight);
  prism.setInputCloud (sensorCloud);
  prism.setInputPlanarHull (tableConvexHull);  
  PointIndices::Ptr indicesOverTheTable (new PointIndices ());
  prism.segment (*indicesOverTheTable);

  // Extraxt indices over the table
  ExtractIndices<PointXYZRGB> extractIndices;
  extractIndices.setInputCloud(sensorCloud);
  extractIndices.setIndices(indicesOverTheTable);
  extractIndices.filter(*cloudOverTheTable);
}

void
downsample(const PointCloud<PointXYZRGB>::Ptr& input, PointCloud<PointXYZRGB>::Ptr& output)
{
  // Create the filtering object: downsample the dataset using a leaf size of 0.5cm
  pcl::VoxelGrid<PointXYZRGB> sor;
  sor.setInputCloud (input);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*output);
}

void 
sensorCallback (const PCLPointCloud2::ConstPtr& sensorInput)
{
  ROS_INFO_ONCE("Callback Called");
  PointCloud<PointXYZRGB>::Ptr sensorCloud(new PointCloud<PointXYZRGB>);
  fromPCLPointCloud2 (*sensorInput,*sensorCloud);

  // Downsample
  PointCloud<PointXYZRGB>::Ptr downsampledCloud (new PointCloud<PointXYZRGB>);
  downsample(sensorCloud, downsampledCloud);
  PCLPointCloud2 downsampledCloudMsg;
  toPCLPointCloud2 (*downsampledCloud, downsampledCloudMsg);
  pubDownsampled.publish(downsampledCloudMsg);

  // Compute integral image normals
  PointCloud<Normal>::Ptr cloudNormals (new PointCloud<Normal>);
  computeNormalsEfficiently(sensorCloud, 2, 0.02f, 10.0f, cloudNormals);
  
  // Segment plane using normals
  ModelCoefficients::Ptr tableCoefficients (new ModelCoefficients ());
  PointIndices::Ptr tableInliers (new PointIndices ());
  if (fitPlaneFromNormals(sensorCloud, cloudNormals, 0.01, 500, tableCoefficients, tableInliers))
  {
    return;
  }
  //pubPlanar.publish();

  // Project plane points (inliers) in model plane
  PointCloud<PointXYZRGB>::Ptr projectedTableCloud (new PointCloud<PointXYZRGB>);
  projectOnPlane(sensorCloud, tableCoefficients, tableInliers, projectedTableCloud);

  // Create a Convex Hull representation of the projected inliers
  PointCloud<PointXYZRGB>::Ptr tableConvexHull(new PointCloud<PointXYZRGB>);
  computeTableConvexHull(projectedTableCloud, tableConvexHull);

  // Extract points over the table's convex hull  
  PointCloud<PointXYZRGB>::Ptr cloudOverTheTable(new PointCloud<PointXYZRGB>);
  extractCloudOverTheTable(sensorCloud, tableConvexHull, 0.0, 0.2, cloudOverTheTable);

  // Clustering objects over the table
  clusterObjects(cloudOverTheTable, 0.02, 100, 25000);
  //pubPlanar.publish();
}

int
main (int argc, char** argv)
{
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("DEBUG ACTIVATED");

  // Initialize ROS
  ros::init (argc, argv, "memoria");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, sensorCallback);

  // Create a ROS publisher
  pubDownsampled = nh.advertise<PCLPointCloud2> ("downsampled", 1);
  pubPlanar = nh.advertise<PCLPointCloud2> ("planar", 1);
  pubObjects = nh.advertise<PCLPointCloud2> ("objects", 1);

  ROS_INFO("Escuchando");

  // Spin
  ros::spin ();
}