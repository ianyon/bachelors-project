//
// Created by ian on 9/30/15.
//

#ifndef BACHELORS_FINAL_PROJECT_VIEWERSPAWNER_H
#define BACHELORS_FINAL_PROJECT_VIEWERSPAWNER_H

#include "base_visualizer.h"
#include "segmentation_visualizer.h"
#include "detection_visualizer.h"

namespace bachelors_final_project
{

// Forward declarations
namespace segmentation
{
class CloudSegmentator;
}
namespace detection
{
class GraspPointDetector;
}


namespace visualization
{
class ViewerSpawner
{
public:
  ViewerSpawner(segmentation::CloudSegmentator *segmentator, detection::GraspPointDetector *detector);

  void spawn();

  void setParams(int normals_count, float normals_size);

  bool show_segmentation_viewer_;
  bool show_detection_viewer_;

  SegmentationVisualizer segmentation_visualizer_;
  DetectionVisualizer detection_visualizer_;

  boost::thread segmentation_thread;
  boost::thread detection_thread;
};

} // namespace visualization
} // namespace bachelors_final_project

#endif //BACHELORS_FINAL_PROJECT_VIEWERSPAWNER_H
