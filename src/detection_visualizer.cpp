//
// Created by ian on 9/30/15.
//

#include "detection_visualizer.h"

#include "grasp_point_detector.h"

using namespace pcl::visualization;

namespace bachelors_final_project
{
visualization::DetectionVisualizer::DetectionVisualizer(detection::GraspPointDetector *detector):
    detector_(detector)
{
}

void visualization::DetectionVisualizer::configureDetectionViewer(PCLVisualizer &viewer)
{
  configureBaseViewer(viewer);
}

void visualization::DetectionVisualizer::visualize()
{
  PCLVisualizer viewer("Detection Viewer");

  configureDetectionViewer(viewer);

    while (!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      visualizeBoundingBox(viewer);
    }
}

void visualization::DetectionVisualizer::visualizeBoundingBox(PCLVisualizer &viewer)
{
  if (detector_->draw_bounding_box_)
  {
    PointCloudColorHandlerCustom<pcl::PointXYZ> white_color(detector_->object_cloud_, 180, 180, 180);
    if (!viewer.updatePointCloud(detector_->object_cloud_, white_color, "new cloud"))
      viewer.addPointCloud<pcl::PointXYZ>(detector_->object_cloud_, white_color, "new cloud");

    // Draw the box
    viewer.removeShape("bounding box");
    viewer.addCube(detector_->bounding_box_.translation,
                    detector_->bounding_box_.rotation,
                    detector_->bounding_box_.max_pt.x - detector_->bounding_box_.min_pt.x,
                    detector_->bounding_box_.max_pt.y - detector_->bounding_box_.min_pt.y,
                    detector_->bounding_box_.max_pt.z - detector_->bounding_box_.min_pt.z,
                    "bounding box");

    detector_->draw_bounding_box_ = false;
  }
}
}