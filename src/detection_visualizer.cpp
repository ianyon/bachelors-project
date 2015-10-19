//
// Created by ian on 9/30/15.
//

#include "detection_visualizer.h"

#include <string>
#include <iostream>

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
  std::cout << "Init detection visualize!" << std::endl;
  PCLVisualizer viewer("Detection Viewer");

  configureDetectionViewer(viewer);

  std::cout << "Ready to loop detection visualizer!" << std::endl;
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);

    boost::mutex::scoped_lock bounding_box_lock(detector_->update_bounding_box_mutex_);
    visualizeBoundingBox(viewer);
    visualizeSampledGrasps(viewer);
    bounding_box_lock.unlock();
  }
}

void visualization::DetectionVisualizer::visualizeBoundingBox(PCLVisualizer &viewer)
{
  if (detector_->draw_bounding_box_)
  {
    PointCloudColorHandlerCustom<Point> white_color(detector_->object_cloud_, 180, 180, 180);
    if (!viewer.updatePointCloud(detector_->object_cloud_, white_color, "object"))
      viewer.addPointCloud<Point>(detector_->object_cloud_, white_color, "object");

    PointCloudColorHandlerCustom<Point> grey_color(detector_->transformed_cloud_, 120, 120, 120);
    if (!viewer.updatePointCloud(detector_->transformed_cloud_, grey_color, "transformed_object"))
      viewer.addPointCloud<Point>(detector_->transformed_cloud_, grey_color, "transformed_object");

    // Draw the box
    viewer.removeShape("bounding box");
    viewer.addCube(detector_->bounding_box_->translation_,
                   detector_->bounding_box_->rotation_,
                   detector_->bounding_box_->max_pt_.x - detector_->bounding_box_->min_pt_.x,
                   detector_->bounding_box_->max_pt_.y - detector_->bounding_box_->min_pt_.y,
                   detector_->bounding_box_->max_pt_.z - detector_->bounding_box_->min_pt_.z,
                   "bounding box");

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    //transform.translate(bounding_box.translation_);
    transform.translate(detector_->bounding_box_->translation_);
    transform.rotate(detector_->bounding_box_->eigen_vectors_);
    viewer.addCoordinateSystem(0.25,transform);

    viewer.removeShape("transformed bounding box");
    viewer.addCube(-0.5f * detector_->bounding_box_->mean_diag_,
                   Eigen::Quaternionf::Identity(),
                   detector_->bounding_box_->max_pt_.x - detector_->bounding_box_->min_pt_.x,
                   detector_->bounding_box_->max_pt_.y - detector_->bounding_box_->min_pt_.y,
                   detector_->bounding_box_->max_pt_.z - detector_->bounding_box_->min_pt_.z,
                   "transformed bounding box");

    Point origin(0.0, 0.0, 0.0);
    Point eigen1, eigen2;
    eigen1.getVector3fMap() = detector_->bounding_box_->eigen_vectors_.col(0);
    eigen2.getVector3fMap() = detector_->bounding_box_->eigen_vectors_.col(1);

    viewer.removeShape("eigen1");
    viewer.addLine<Point>(origin, eigen1, "eigen1");
    viewer.removeShape("eigen2");
    viewer.addLine<Point>(origin, eigen2, "eigen2");

    detector_->draw_bounding_box_ = false;
  }
}

void visualization::DetectionVisualizer::visualizeSampledGrasps(PCLVisualizer viewer)
{
  if (detector_->draw_sampled_grasps_)
  {
    Point middle;
    middle.getVector4fMap() = detector_->bounding_box_->centroid_;

    viewer.removeShape("real plane");
    viewer.addPlane(*(detector_->table_plane_), middle.x, middle.y, middle.z, "real plane");

    PointCloudPtr sampled_side_grasps = detector_->getSampledSideGrasps();

    PointCloudColorHandlerCustom<Point> red_color(sampled_side_grasps, 255, 0, 0);
    if (!viewer.updatePointCloud(sampled_side_grasps, red_color, "sampled side cloud"))
      viewer.addPointCloud<Point>(sampled_side_grasps, red_color, "sampled side cloud");

    PointCloudPtr sampled_top_grasps = detector_->getSampledTopGrasps();

    PointCloudColorHandlerCustom<Point> new_red_color(sampled_top_grasps, 255, 0, 0);
    if (!viewer.updatePointCloud(sampled_top_grasps, new_red_color, "sampled top cloud"))
      viewer.addPointCloud<Point>(sampled_top_grasps, new_red_color, "sampled top cloud");

    visualizePoint(middle, 0, 0, 255, "centroid_", viewer);

    detector_->draw_sampled_grasps_ = false;
  }
}

void visualization::DetectionVisualizer::visualizePoint(Point point, int red, int green, int blue,
                                                        string name, PCLVisualizer viewer)
{
  PointCloudPtr cloud(new PointCloudT);
  cloud->push_back(point);

  PointCloudColorHandlerCustom<Point> color(cloud, red, green, blue);
  if (!viewer.updatePointCloud(cloud, color, name))
    viewer.addPointCloud<Point>(cloud, color, name);
}
} // namespace bachelors_final_project
