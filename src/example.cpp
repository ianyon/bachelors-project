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

void parameterCallback(ParametersConfig &cfg, uint32_t level)
{
  if (cfg.defaultParams)
  {
    ROS_INFO("Reset parameters to default");
    cfg = cfg.__getDefault__();
  }

  data_handler->updateConfig(cfg);

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
}
