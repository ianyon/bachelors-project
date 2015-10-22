//
// Created by ian on 10/21/15.
//

#include "visualization_thread.h"

#include <ros/init.h>

#include "segmentation_visualizer.h"
#include "detection_visualizer.h"
#include "containers.h"

using namespace pcl::visualization;

using std::string;

namespace bachelors_final_project
{

void visualization::VisualizationThread::addSegmentationVisualizer(segmentation::CloudSegmentator &segmentator)
{
  visualizers.push_back(BaseVisualizerPtr(new SegmentationVisualizer(segmentator)));
}

void visualization::VisualizationThread::addDetectionVisualizer(detection::GraspPointDetector &detector)
{
  visualizers.push_back(BaseVisualizerPtr(new DetectionVisualizer(detector)));
}

void visualization::VisualizationThread::visualizationLoop()
{
  ROS_INFO("Initiated visualization thread!");
  while (ros::ok() && visualizers.size() > 0)
  {
    for (size_t i = 0; i < visualizers.size(); ++i)
    {
      if (visualizers[i]->wasStopped())
      {
        visualizers.erase(visualizers.begin() + i);
        break;
      }
      visualizers[i]->computeSpinOnce();
    }
  }
}

void visualization::VisualizationThread::setParams(int normals_count, float normals_size)
{
  VisualizerParams params(normals_count, normals_size);
  BOOST_FOREACH(BaseVisualizerPtr viz, visualizers)
          viz->setParams(params);
}

void visualization::VisualizationThread::start()
{
  boost::thread(&VisualizationThread::visualizationLoop, this);
}
} // namespace bachelors_final_project
