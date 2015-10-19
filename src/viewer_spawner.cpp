//
// Created by ian on 9/30/15.
//
#include "viewer_spawner.h"

#include <iostream>

namespace bachelors_final_project
{
visualization::ViewerSpawner::ViewerSpawner(segmentation::CloudSegmentator *segmentator,
                                            detection::GraspPointDetector *detector):
    segmentation_visualizer_(segmentator), detection_visualizer_(detector)
{
  show_segmentation_viewer_ = true;
  show_detection_viewer_ = false;
}

void visualization::ViewerSpawner::spawn()
{
  if(show_segmentation_viewer_)
  {
    std::cout << "Spawning Segmentation visualizer!" << std::endl;
    segmentation_thread = boost::thread(&SegmentationVisualizer::visualize, &segmentation_visualizer_);
    segmentation_thread.detach();
  }

  if (show_detection_viewer_)
  {
    std::cout << "Spawning Detection visualizer!" << std::endl;
    detection_thread = boost::thread(&DetectionVisualizer::visualize, &detection_visualizer_);
    detection_thread.detach();
  }
}

void visualization::ViewerSpawner::setParams(int normals_count, float normals_size)
{
  segmentation_visualizer_.normals_count_ = normals_count;
  segmentation_visualizer_.normals_size_ = normals_size;
}
} // namespace bachelors_final_project