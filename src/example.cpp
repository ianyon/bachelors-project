// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>

// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Project includes
#include <bachelors_final_project/ParametersConfig.h>
#include "../include/data_handler.h"
#include "../include/data_visualizer.h"

DataVisualizer *visualizer;
DataHandler *data_handler;

// Reset all parameters
bool defaultParams;

void resetToDefaults(bachelors_final_project::ParametersConfig &cfg)
{
  ROS_INFO("Reset parameters to default");
  cfg = cfg.__getDefault__();
  /*
  // Cropping
  cfg.scaleParam = cfg.__getDefault__().scaleParam;
  cfg.xTranslateParam = cfg.__getDefault__().xTranslateParam;
  cfg.yTranslateParam = cfg.__getDefault__().yTranslateParam;

  // Gaussian Smoothing
  cfg.gaussianSigmaParam = cfg.__getDefault__().gaussianSigmaParam;
  cfg.gaussianSearchRadiusParam = cfg.__getDefault__().gaussianSearchRadiusParam;

  // Compute normals efficiently
  cfg.normalEstimationMethodParam = cfg.__getDefault__().normalEstimationMethodParam;
  cfg.maxDepthChangeFactorParam = cfg.__getDefault__().maxDepthChangeFactorParam;
  cfg.useDepthDependentSmoothingParam = cfg.__getDefault__().useDepthDependentSmoothingParam;
  cfg.normalSmoothingSizeParam = cfg.__getDefault__().normalSmoothingSizeParam;

  // Fit Plane From Normals
  cfg.normalDistanceWeightParam = cfg.__getDefault__().normalDistanceWeightParam;
  cfg.originDistanceParam = cfg.__getDefault__().originDistanceParam;
  cfg.maxIterationsParam = cfg.__getDefault__().maxIterationsParam;
  cfg.distanceThresholdParam = cfg.__getDefault__().distanceThresholdParam;
  cfg.optimizeCoefficientsParam = cfg.__getDefault__().optimizeCoefficientsParam;
  cfg.probabilityParam = cfg.__getDefault__().probabilityParam;
  cfg.sampleMaxDistanceParam = cfg.__getDefault__().sampleMaxDistanceParam;
  cfg.useSpecificPlaneParam = cfg.__getDefault__().useSpecificPlaneParam;
  cfg.planeXParam = cfg.__getDefault__().planeXParam;
  cfg.planeYParam = cfg.__getDefault__().planeYParam;
  cfg.planeZParam = cfg.__getDefault__().planeZParam;
  cfg.epsAngleParam = cfg.__getDefault__().epsAngleParam;

  // Cloud Over The Table
  cfg.minHeightParam = cfg.__getDefault__().minHeightParam;
  cfg.maxHeightParam = cfg.__getDefault__().maxHeightParam;

  // Euclidean Cluster
  cfg.clusterTolerance = cfg.clusterTolerance;
  cfg.minClusterSize = cfg.minClusterSize;
  cfg.maxClusterSize = cfg.maxClusterSize;

  // Visualizer
  cfg.normalsCountParam = cfg.__getDefault__().normalsCountParam;
  cfg.normalsSizeParam = cfg.__getDefault__().normalsSizeParam;

  // Reset all parameters
  cfg.defaultParams = cfg.__getDefault__().defaultParams;
  */
}

void parameterCallback(bachelors_final_project::ParametersConfig &cfg, uint32_t level)
{
  if (cfg.defaultParams)
    resetToDefaults(cfg);

  // Cropping
  data_handler->scale = cfg.scaleParam;
  data_handler->xTranslate = cfg.xTranslateParam;
  data_handler->yTranslate = cfg.yTranslateParam;

  // Gaussian Smoothing
  data_handler->gaussianSigma = cfg.gaussianSigmaParam;
  data_handler->gaussianSearchRadius = cfg.gaussianSearchRadiusParam;

  // Compute normals efficiently
  data_handler->normalEstimationMethod = cfg.normalEstimationMethodParam;
  data_handler->maxDepthChangeFactor = cfg.maxDepthChangeFactorParam;
  data_handler->useDepthDependentSmoothing = cfg.useDepthDependentSmoothingParam;
  data_handler->normalSmoothingSize = cfg.normalSmoothingSizeParam;

  // Fit Plane From Normals
  data_handler->normalDistanceWeight = cfg.normalDistanceWeightParam;
  data_handler->originDistance = cfg.originDistanceParam;
  data_handler->maxIterations = cfg.maxIterationsParam;
  data_handler->distanceThreshold = cfg.distanceThresholdParam;
  data_handler->optimizeCoefficients = cfg.optimizeCoefficientsParam;
  data_handler->probability = cfg.probabilityParam;
  data_handler->sampleMaxDistance = cfg.sampleMaxDistanceParam;
  data_handler->useSpecificPlane = cfg.useSpecificPlaneParam;
  data_handler->planeX = cfg.planeXParam;
  data_handler->planeY = cfg.planeYParam;
  data_handler->planeZ = cfg.planeZParam;
  data_handler->epsAngle = cfg.epsAngleParam;

  // Cloud Over The Table
  data_handler->minHeight = cfg.minHeightParam;
  data_handler->maxHeight = cfg.maxHeightParam;

  // Euclidean Cluster
  data_handler->cluster_tolerance_ = cfg.clusterTolerance;
  data_handler->min_cluster_size_ = cfg.minClusterSize;
  data_handler->max_cluster_size_ = cfg.maxClusterSize;

  // Visualizer
  visualizer->normals_count_ = cfg.normalsCountParam;
  visualizer->normals_size_ = cfg.normalsSizeParam;

  // Reset all parameters
  defaultParams = cfg.defaultParams;

  ROS_WARN("Done Reconfigure Request");
}

int main (int argc, char** argv)
{
  // Delete parameters to start in clean state
  //ros::param::del("/bachelors_final_project");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("DEBUG ACTIVATED");

  // Initialize ROS
  ros::init (argc, argv, "bachelors_final_project");
  ros::NodeHandle nh;

  data_handler = new DataHandler(nh);
  visualizer = new DataVisualizer(*data_handler);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, &DataHandler::sensorCallback, data_handler);

  // Create Dynamic reconfigure server
  dynamic_reconfigure::Server<bachelors_final_project::ParametersConfig> server;
  dynamic_reconfigure::Server<bachelors_final_project::ParametersConfig>::CallbackType f;

  // Bind callback function to update values
  f = boost::bind(&parameterCallback, _1, _2);
  server.setCallback(f);

  //Start visualizer thread
  boost::thread workerThread(&DataVisualizer::visualize, visualizer);
  workerThread.detach();

  data_handler->plane_updated_ = false;
  data_handler->point_clouds_updated_ = false;

  ROS_INFO("Escuchando");

  // Spin
  ros::spin();        // Handle ROS events

  // Delete parameters to start in clean state
  //ros::param::del("/bachelors_final_project");
}
