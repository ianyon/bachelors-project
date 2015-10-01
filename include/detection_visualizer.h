//
// Created by ian on 9/30/15.
//

#ifndef BACHELORS_FINAL_PROJECT_DETECTIONVISUALIZER_H
#define BACHELORS_FINAL_PROJECT_DETECTIONVISUALIZER_H

#include "base_visualizer.h"

#include <pcl/visualization/pcl_visualizer.h>

namespace bachelors_final_project
{
// Forward declarations
namespace detection
{
class GraspPointDetector;
}

namespace visualization
{
class DetectionVisualizer
    : public BaseVisualizer
{

public:
  DetectionVisualizer()
  { }

  DetectionVisualizer(detection::GraspPointDetector *);

  void visualize();

  void configureDetectionViewer(pcl::visualization::PCLVisualizer&);

  void visualizeBoundingBox(pcl::visualization::PCLVisualizer&);

  detection::GraspPointDetector *detector_;
};

} // namespace visualization
} // namespace bachelors_final_project
#endif //BACHELORS_FINAL_PROJECT_DETECTIONVISUALIZER_H
