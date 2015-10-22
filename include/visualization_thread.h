//
// Created by ian on 10/21/15.
//

#ifndef BACHELORS_FINAL_PROJECT_VISUALIZATIONTHREAD_H
#define BACHELORS_FINAL_PROJECT_VISUALIZATIONTHREAD_H

#include <string>

#include <pcl/visualization/pcl_visualizer.h>

#include "cloud_segmentator.h"
#include "grasp_point_detector.h"
#include "base_visualizer.h"

namespace bachelors_final_project
{
namespace visualization
{

class VisualizationThread
{
public:

  void visualizationLoop();

  void addSegmentationVisualizer(segmentation::CloudSegmentator &segmentator);

  void addDetectionVisualizer(detection::GraspPointDetector &detector);

  void setParams(int normals_count, float normals_size);

  void start();

  std::vector<BaseVisualizerPtr> visualizers;
};

} // namespace visualization
} // namespace bachelors_final_project
#endif //BACHELORS_FINAL_PROJECT_VISUALIZATIONTHREAD_H
