#if defined(__clang__)
#pragma message "\n\n\nCLANG\n\n\n"
#else
#pragma message "\n\n\nNOT CLANG\n\n\n"
#endif

// ROS includes
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>

// Project includes
#include <bachelors_final_project/ParametersConfig.h>
#include "../include/data_handler.h"
#include "../include/data_visualizer.h"

namespace bachelors_final_project
{
DataVisualizer *visualizer;
DataHandler *data_handler;

void parameterCallback(bachelors_final_project::ParametersConfig &cfg, uint32_t level)
{
  if (cfg.defaultParams)
  {
    ROS_INFO("Reset parameters to default");
    cfg = cfg.__getDefault__();
  }

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
  visualizer->normals_size_ = (float) cfg.normalsSizeParam;

  ROS_WARN("Done Reconfigure Request");
}

}

int main (int argc, char** argv)
{
  using namespace bachelors_final_project;
  // Delete parameters to start in clean state
  //ros::param::del("/bachelors_final_project");

  /*if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("DEBUG ACTIVATED");*/

  // Initialize ROS
  ros::init (argc, argv, "bachelors_final_project");
  ros::NodeHandle nh;

  data_handler = new DataHandler(nh);
  visualizer = new DataVisualizer(*data_handler);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, &DataHandler::sensorCallback, data_handler);

  // Create Dynamic reconfigure server
  dynamic_reconfigure::Server<ParametersConfig> server;
  // Bind callback function to update values
  dynamic_reconfigure::Server<ParametersConfig>::CallbackType f = boost::bind(&parameterCallback, _1, _2);
  server.setCallback(f);

  //Start visualizer thread
  boost::thread visualizationThread(&DataVisualizer::visualize, visualizer);
  visualizationThread.detach();

  ROS_INFO("Escuchando");

  // Spin
  while(ros::ok())
  {
    data_handler->execute();        // Do Heavy processing
    ros::spinOnce();                // Handle ROS events
  }
  // Delete parameters to start in clean state
  //ros::param::del("/bachelors_final_project");
}
