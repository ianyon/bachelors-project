#if defined(__clang__)
#pragma message "\n\n\nCLANG\n\n\n"
#else
#pragma message "\n\n\nNOT CLANG\n\n\n"
#endif

#include <dynamic_reconfigure/server.h>

#include <bachelors_final_project/ParametersConfig.h>
#include <base_visualizer.h>
#include <viewer_spawner.h>
#include "cloud_segmentator.h"
#include "grasp_point_detector.h"

namespace bachelors_final_project
{

void parameterCallback(ParametersConfig &cfg, uint32_t level,
                       segmentation::CloudSegmentator *data_handler, visualization::ViewerSpawner *visualizer)
{
  if (cfg.defaultParams)
  {
    ROS_INFO("Reset parameters to default");
    cfg = cfg.__getDefault__();
  }

  data_handler->updateConfig(cfg);

  // Visualizer
  visualizer->setParams(cfg.normalsCountParam, (float) cfg.normalsSizeParam);

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

  /*if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("DEBUG ACTIVATED");*/

  // Initialize ROS
  ros::init(argc, argv, "bachelors_final_project");
  ros::NodeHandle nh;

  // CloudSegmentator needs to be a pointer because mutex cannot be copied
  CloudSegmentator *segmentator = new CloudSegmentator(nh);
  gpd::GraspPointDetector detector(nh);
  viz::ViewerSpawner spawner(segmentator, &detector);


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, &CloudSegmentator::sensorCallback, segmentator);

  // Create Dynamic reconfigure server
  dynamic_reconfigure::Server<ParametersConfig> server;
  // Bind callback function to update values
  dynamic_reconfigure::Server<ParametersConfig>::CallbackType f = boost::bind(
      &parameterCallback, _1, _2, segmentator, &spawner);
  server.setCallback(f);

  //Start visualization
  spawner.spawn();

  ROS_INFO("Escuchando");

  while(sub.getNumPublishers () < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();
  }

  // Spin
  while (ros::ok())
  {
    ros::spinOnce();                // Handle ROS events

    segmentator->execute();        // Do Heavy processing

    if (segmentator->cloud_cluster_vector_.size() == 0)
    {
      //ROS_INFO("No clusters, continue searching");
      continue;
    }

    size_t selected_cluster_index;
    size_t max_size = 0;

    for (size_t i = 0; i < segmentator->cloud_cluster_vector_.size(); i++)
    {
      if (segmentator->cloud_cluster_vector_[i]->size() > max_size)
        selected_cluster_index = i;
    }

    int cluster_size = segmentator->cloud_cluster_vector_[selected_cluster_index]->size();
    ROS_INFO("Using cluster with %d points", cluster_size);

    detector.detect(segmentator->getCluster(0), segmentator->getTable());
  }
}

