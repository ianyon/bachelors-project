// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>

// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

//Common
#include <pcl/common/gaussian.h>

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

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

// Project includes
#include <bachelors_final_project/ParametersConfig.h>
#include "include/data_handler.h"
#include "include/data_visualizer.h"

void parameterCallback(bachelors_final_project::ParametersConfig &cfg, uint32_t level)
{

  cfg.__getDefault__().scaleParam
  ROS_INFO("Parameters modified");
  if (cfg.defaultParams)
  {
    ROS_INFO("Reset parameters to default");
    // Cropping
    cfg.scaleParam = 2;
    cfg.xTranslateParam = 0;
    cfg.yTranslateParam = 10;

    // Passthrough
    cfg.yLimitParam = 0.18;
    cfg.xLimitParam = 0.405;
    cfg.zLimitParam = 1.575;

    // Gaussian Smoothing
    cfg.gaussianSigmaParam = 2.0;
    cfg.gaussianSearchRadiusParam = 0.005;

    // Compute normals efficiently
    cfg.normalEstimationMethodParam = 4;
    cfg.maxDepthChangeFactorParam = 0.02;
    cfg.useDepthDependentSmoothingParam = false;
    cfg.normalSmoothingSizeParam = 10.0;

    // Fit Plane From Normals
    cfg.normalDistanceWeightParam = 0.1;
    cfg.minAngleParam = 0;
    cfg.maxAngleParam = 2*M_PI;
    cfg.originDistanceParam = 150.0;
    cfg.maxIterationsParam = 500;
    cfg.distanceThresholdParam = 0.3;
    cfg.optimizeCoefficientsParam = true;
    cfg.probabilityParam = 0.99;
    cfg.sampleMaxDistanceParam = 200.0;
    cfg.useSpecificPlaneParam = false;
    cfg.planeXParam = 0.0;
    cfg.planeYParam = 1.0;
    cfg.planeZParam = 0.0;
    cfg.epsAngleParam = 2*M_PI;

    // Cloud Over The Table
    cfg.minHeightParam = 0.0;
    cfg.maxHeightParam = 0.2;

    // Visualizer
    cfg.vizNormalsCountParam = 50;

    // Reset all parameters
    cfg.defaultParams = false;
  }

  // Cropping
  scale = cfg.scaleParam;
  xTranslate = cfg.xTranslateParam;
  yTranslate = cfg.yTranslateParam;

  // Passthrough
  yLimit = cfg.yLimitParam;
  xLimit = cfg.xLimitParam;
  zLimit = cfg.zLimitParam;

  // Gaussian Smoothing
  gaussianSigma = cfg.gaussianSigmaParam;
  gaussianSearchRadius = cfg.gaussianSearchRadiusParam;

  // Compute normals efficiently
  normalEstimationMethod = cfg.normalEstimationMethodParam;
  maxDepthChangeFactor = cfg.maxDepthChangeFactorParam;
  useDepthDependentSmoothing = cfg.useDepthDependentSmoothingParam;
  normalSmoothingSize = cfg.normalSmoothingSizeParam;

  // Fit Plane From Normals
  normalDistanceWeight = cfg.normalDistanceWeightParam;
  minAngle = cfg.minAngleParam;
  maxAngle = cfg.maxAngleParam;
  originDistance = cfg.originDistanceParam;
  maxIterations = cfg.maxIterationsParam;
  distanceThreshold = cfg.distanceThresholdParam;
  optimizeCoefficients = cfg.optimizeCoefficientsParam;
  probability = cfg.probabilityParam;
  sampleMaxDistance = cfg.sampleMaxDistanceParam;
  useSpecificPlane = cfg.useSpecificPlaneParam;
  planeX = cfg.planeXParam;
  planeY = cfg.planeYParam;
  planeZ = cfg.planeZParam;
  epsAngle = cfg.epsAngleParam;

  // Cloud Over The Table
  minHeight = cfg.minHeightParam;
  maxHeight = cfg.maxHeightParam;

  // Visualizer
  vizNormalsCount = cfg.vizNormalsCountParam;

  // Reset all parameters
  defaultParams = cfg.defaultParams;

  ROS_INFO("Reconfigure Request");
}

int
main (int argc, char** argv)
{
  // Delete parameters to start in clean state
  ros::param::del("/bachelors_final_project");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("DEBUG ACTIVATED");

  // Initialize ROS
  ros::init (argc, argv, "bachelors_final_project");
  ros::NodeHandle nh;

  // Create a new NodeExample object.
  DataHandler *data_handler = new DataHandler(nh);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, &DataHandler::sensorCallback, data_handler);

  // Create Dynamic reconfigure server
  dynamic_reconfigure::Server<bachelors_final_project::ParametersConfig> server;
  dynamic_reconfigure::Server<bachelors_final_project::ParametersConfig>::CallbackType f;

  // Bind callback function to update values
  f = boost::bind(&DataHandler::parameterCallback, data_handler, _1, _2);
  server.setCallback(f);

  /*
  ROS_INFO("Init Params= gaussianSigma:%g "
           "gaussianSearchRadius:%g "
           "normalEstimationMethod:%i"
           "normalDistanceWeight:%g "
           "distanceThreshold:%g "
           "maxDepthChangeFactor:%g "
           "normalSmoothingSize:%g "
           "minHeight:%g "
           "maxHeight:%g "
           "useDepthDependentSmoothing:%s "
           "defaultParams:%s",
           gaussianSigma,
           gaussianSearchRadius,
           normalEstimationMethod,
           normalDistanceWeight,
           distanceThreshold,
           maxDepthChangeFactor,
           normalSmoothingSize,
           minHeight,
           maxHeight,
           useDepthDependentSmoothing?"True":"False",
           defaultParams?"True":"False");
           */

  DataVisualizer *visualizer = new DataVisualizer(data_handler);

  //Start visualizer thread
  boost::thread workerThread(&DataVisualizer::visualize, visualizer);
  workerThread.detach();

  ROS_INFO("Escuchando");

  // Spin
  ros::spin();        // Handle ROS events

  // Delete parameters to start in clean state
  ros::param::del("/bachelors_final_project");
}
