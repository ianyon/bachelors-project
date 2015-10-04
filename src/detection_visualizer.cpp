//
// Created by ian on 9/30/15.
//

#include "detection_visualizer.h"

#include <string>

#include "grasp_point_detector.h"

using namespace pcl::visualization;
using std::string;

namespace bachelors_final_project
{

visualization::DetectionVisualizer::DetectionVisualizer(detection::GraspPointDetector *detector) :
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
    visualizeSampledGrasps(viewer);
  }
}

void visualization::DetectionVisualizer::visualizeBoundingBox(PCLVisualizer &viewer)
{
  if (detector_->draw_bounding_box_)
  {
    PointCloudColorHandlerCustom<PointT> white_color(detector_->object_cloud_, 180, 180, 180);
    if (!viewer.updatePointCloud(detector_->object_cloud_, white_color, "new cloud"))
      viewer.addPointCloud<PointT>(detector_->object_cloud_, white_color, "new cloud");

    // Draw the box
    viewer.removeShape("bounding box");
    viewer.addCube(detector_->bounding_box_.translation,
                   detector_->bounding_box_.rotation,
                   detector_->bounding_box_.max_pt.x - detector_->bounding_box_.min_pt.x,
                   detector_->bounding_box_.max_pt.y - detector_->bounding_box_.min_pt.y,
                   detector_->bounding_box_.max_pt.z - detector_->bounding_box_.min_pt.z,
                   "bounding box");

    /*viewer.removeShape("bounding");
    viewer.addCube(detector_->bounding_box_.min_pt.x, detector_->bounding_box_.max_pt.x,
                   detector_->bounding_box_.min_pt.y, detector_->bounding_box_.max_pt.y,
                   detector_->bounding_box_.min_pt.z, detector_->bounding_box_.max_pt.z,
                   0.2, 0.2, 0.2, "bounding");*/

    detector_->draw_bounding_box_ = false;
  }
}

void visualization::DetectionVisualizer::visualizeSampledGrasps(PCLVisualizer viewer)
{
  if (detector_->draw_sampled_grasps_)
  {
    PointT middle;
    middle.getVector4fMap() = detector_->bounding_box_.centroid;

    //viewer.removeShape("real plane");
    viewer.addPlane(*(detector_->table_plane_), middle.x, middle.y, middle.z, "real plane");
    //viewer.removeShape("where plane");
    //viewer.addPlane(*(detector_->table_plane_), "where plane");


    PointCloudTPtr sampled_side_grasps = detector_->getSampledSideGrasps();

    PointCloudColorHandlerCustom<PointT> red_color(sampled_side_grasps, 255, 0, 0);
    if (!viewer.updatePointCloud(sampled_side_grasps, red_color, "sampled side cloud"))
      viewer.addPointCloud<PointT>(sampled_side_grasps, red_color, "sampled side cloud");

    //PointT mean_diag;
    //mean_diag.getVector3fMap() = detector_->bounding_box_.mean_diag;

    //visualizePoint(mean_diag, 0, 255, 255, "mean_diag", viewer);
    visualizePoint(middle, 0, 0, 255, "centroid", viewer);

    detector_->draw_sampled_grasps_ = false;
  }
}

void visualization::DetectionVisualizer::visualizePoint(PointT point, int red, int green, int blue,
                                                        string name, PCLVisualizer viewer)
{
  PointCloudTPtr cloud(new PointCloudT);
  cloud->push_back(point);

  PointCloudColorHandlerCustom<PointT> color(cloud, red, green, blue);
  if (!viewer.updatePointCloud(cloud, color, name))
    viewer.addPointCloud<PointT>(cloud, color, name);
}
} // namespace bachelors_final_project