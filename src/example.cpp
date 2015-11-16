#if defined(__clang__)
#pragma message "\n\n\nCLANG\n\n\n"
#else
#pragma message "\n\n\nNOT CLANG\n\n\n"
#endif

#include <string>
#include <stdio.h>
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
                       visualization::VisualizationThread *viz_thread,
                       detection::GraspPointDetector *detector,
                       int *cluster_selector)
{
  if (cfg.defaultParams)
  {
    ROS_INFO("Reset parameters to default");
    cfg = cfg.__getDefault__();
  }

  data_handler->updateConfig(cfg);

  // Visualizer
  viz_thread->setParams(cfg.normalsCountParam, (float) cfg.normalsSizeParam);

  detector->setParams(cfg.standoff);

  *cluster_selector = cfg.clusterSelector;

  // Always abort computation and reconfigure things
  ros::param::set("/bachelors_final_project/reconfigure", true);

  ROS_WARN("Done Reconfigure Request");
}

void openGUIIfAsked(visualization::VisualizationThread &viz_thread,
                    segmentation::CloudSegmentator &segmentator,
                    detection::GraspPointDetector &detector)
{
  ros::NodeHandle priv_nh("~");
  bool segmentation_visualizer;
  if (priv_nh.getParam("segmentation_visualizer", segmentation_visualizer) && segmentation_visualizer)
    viz_thread.addSegmentationVisualizer(segmentator);

  bool detection_visualizer;
  if (priv_nh.getParam("detection_visualizer", detection_visualizer) && detection_visualizer)
    viz_thread.addDetectionVisualizer(detector);

  bool reconfigure_gui;
  if (priv_nh.getParam("reconfigure_gui", reconfigure_gui) && reconfigure_gui)
    system("rosrun rqt_reconfigure rqt_reconfigure /bachelors_final_project 2>&1 &");
}

} // namespace bachelors_final_project


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
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  std_srvs::Empty srv;
  if (client.call(srv)) ROS_INFO("Unpaused physics");
  else
    ROS_ERROR("Failed to call service /gazebo/unpause_physics");

  tf::TransformListener tf_listener(ros::Duration(60));
  viz::VisualizationThread viz_thread(tf_listener);

  // These objects cannot be copied because they contain a mutex
  CloudSegmentator segmentator(nh, tf_listener);
  gpd::GraspPointDetector detector(nh, tf_listener);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe(KINECT_TOPIC, 1, &CloudSegmentator::sensorCallback, &segmentator);
  ROS_INFO("Topic: %s", sub.getTopic().c_str());

  int cluster_selector = 3;

  // Create Dynamic reconfigure server
  dynamic_reconfigure::Server<ParametersConfig> server;
  // Bind callback function to update values
  server.setCallback(boost::bind(&parameterCallback, _1, _2, &segmentator, &viz_thread, &detector, &cluster_selector));

  openGUIIfAsked(viz_thread, segmentator, detector);

  //Start visualization (if there are visualizers)
  viz_thread.start();

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("Escuchando");
  while (ros::ok())
  {
    segmentator.execute();

    if (segmentator.noNewProcessedData()) continue;

    std::sort(segmentator.getClusters().begin(), segmentator.getClusters().end(), cloudSizeComparator);
    int cluster_index = getNBiggerIndex(segmentator.getClusters().size(), cluster_selector);
    ROS_INFO("Using cluster with %lu points", segmentator.getCluster(cluster_index)->size());
    detector.setTable(segmentator.getTableCloud(), segmentator.getTable());
    detector.detect(segmentator.getCluster(cluster_index));
    ROS_INFO("Finished loop\n");
  }
}

