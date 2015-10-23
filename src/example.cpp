#if defined(__clang__)
#pragma message "\n\n\nCLANG\n\n\n"
#else
#pragma message "\n\n\nNOT CLANG\n\n\n"
#endif

#include <iostream>

#include <dynamic_reconfigure/server.h>

#include <std_srvs/Empty.h>

#include "bachelors_final_project/ParametersConfig.h"
#include "base_visualizer.h"
#include "visualization_thread.h"
#include "utils.h"

namespace bachelors_final_project
{

void parameterCallback(ParametersConfig &cfg, uint32_t level,
                       segmentation::CloudSegmentator *data_handler,
                       visualization::VisualizationThread *viz_thread)
{
  if (cfg.defaultParams)
  {
    ROS_INFO("Reset parameters to default");
    cfg = cfg.__getDefault__();
  }

  data_handler->updateConfig(cfg);

  // Visualizer
  viz_thread->setParams(cfg.normalsCountParam, (float) cfg.normalsSizeParam);

  ROS_WARN("Done Reconfigure Request");
}

}

int main(int argc, char **argv)
{
  using namespace bachelors_final_project;
  using namespace bachelors_final_project::segmentation;
  namespace gpd = bachelors_final_project::detection;
  namespace viz = bachelors_final_project::visualization;

  // Delete parameters to start in clean state
  //ros::param::del("/bachelors_final_project");

  /*if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();*/

  // Initialize ROS
  ros::init(argc, argv, "bachelors_final_project");
  ros::NodeHandle nh("~");

  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  std_srvs::Empty srv;
  if (client.call(srv)) ROS_INFO("Unpaused physics");
  else
    ROS_ERROR("Failed to call service /gazebo/unpause_physics");
  ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");

  // CloudSegmentator needs to be a pointer because mutex cannot be copied
  CloudSegmentator segmentator(nh);
  gpd::GraspPointDetector detector(nh);

  viz::VisualizationThread viz_thread;

  if (!nh.hasParam("no_segmentation_visualizer"))
    viz_thread.addSegmentationVisualizer(segmentator);
  if (!nh.hasParam("no_detection_visualizer"))
    viz_thread.addDetectionVisualizer(detector);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe(KINECT_TOPIC, 1, &CloudSegmentator::sensorCallback, &segmentator);

  ROS_INFO("Topic: %s", sub.getTopic().c_str());

  // Create Dynamic reconfigure server
  dynamic_reconfigure::Server<ParametersConfig> server;
  // Bind callback function to update values
  server.setCallback(boost::bind(&parameterCallback, _1, _2, &segmentator, &viz_thread));

  //Start visualization
  viz_thread.start();

  ROS_INFO("Escuchando");
  while (ros::ok())
  {
    ros::spinOnce();                // Handle ROS events
    segmentator.execute();        // Do Heavy processing

    if (segmentator.noNewProcessedData()) continue;

    std::sort (segmentator.getClusters().begin()+4, segmentator.getClusters().end(), cloudSize);
    int cluster_index = segmentator.getClusters().size()>1?1:0;
    ROS_INFO("Using cluster with %lu points", segmentator.getCluster(cluster_index)->size());
    detector.detect(segmentator.getCluster(cluster_index), segmentator.getTable());
  }

  std_srvs::Empty srv2;
  if (client2.call(srv2)) ROS_INFO("Paused physics");
  else
    ROS_ERROR("Failed to call service /gazebo/pause_physics");
}

