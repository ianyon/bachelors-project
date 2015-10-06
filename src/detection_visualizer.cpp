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
    viewer.spinOnce();

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
    PointCloudColorHandlerCustom<PointT> white_color(detector_->object_cloud_, 180, 180, 180);
    if (!viewer.updatePointCloud(detector_->object_cloud_, white_color, "object"))
      viewer.addPointCloud<PointT>(detector_->object_cloud_, white_color, "object");

    PointCloudColorHandlerCustom<PointT> grey_color(detector_->transformed_cloud_, 120, 120, 120);
    if (!viewer.updatePointCloud(detector_->transformed_cloud_, grey_color, "transformed_object"))
      viewer.addPointCloud<PointT>(detector_->transformed_cloud_, grey_color, "transformed_object");

    // Draw the box
    viewer.removeShape("bounding box");
    viewer.addCube(detector_->bounding_box_.translation,
                   detector_->bounding_box_.rotation,
                   detector_->bounding_box_.max_pt.x - detector_->bounding_box_.min_pt.x,
                   detector_->bounding_box_.max_pt.y - detector_->bounding_box_.min_pt.y,
                   detector_->bounding_box_.max_pt.z - detector_->bounding_box_.min_pt.z,
                   "bounding box");

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    //transform.translate(bounding_box.translation);
    transform.translate(detector_->bounding_box_.translation);
    transform.rotate(detector_->bounding_box_.eigen_vectors);
    viewer.addCoordinateSystem(0.25,transform);

    viewer.removeShape("transformed bounding box");
    viewer.addCube(-0.5f * detector_->bounding_box_.mean_diag,
                   Eigen::Quaternionf::Identity(),
                   detector_->bounding_box_.max_pt.x - detector_->bounding_box_.min_pt.x,
                   detector_->bounding_box_.max_pt.y - detector_->bounding_box_.min_pt.y,
                   detector_->bounding_box_.max_pt.z - detector_->bounding_box_.min_pt.z,
                   "transformed bounding box");

    PointT origin(0.0, 0.0, 0.0);
    PointT eigen1, eigen2;
    eigen1.getVector3fMap() = detector_->bounding_box_.eigen_vectors.col(0);
    eigen2.getVector3fMap() = detector_->bounding_box_.eigen_vectors.col(1);

    viewer.removeShape("eigen1");
    viewer.addLine<PointT>(origin, eigen1, "eigen1");
    viewer.removeShape("eigen2");
    viewer.addLine<PointT>(origin, eigen2, "eigen2");

    detector_->draw_bounding_box_ = false;
  }
}

void visualization::DetectionVisualizer::visualizeSampledGrasps(PCLVisualizer viewer)
{
  if (detector_->draw_sampled_grasps_)
  {
    PointT middle;
    middle.getVector4fMap() = detector_->bounding_box_.centroid;

    viewer.removeShape("real plane");
    viewer.addPlane(*(detector_->table_plane_), middle.x, middle.y, middle.z, "real plane");

    PointCloudTPtr sampled_side_grasps = detector_->getSampledSideGrasps();

    PointCloudColorHandlerCustom<PointT> red_color(sampled_side_grasps, 255, 0, 0);
    if (!viewer.updatePointCloud(sampled_side_grasps, red_color, "sampled side cloud"))
      viewer.addPointCloud<PointT>(sampled_side_grasps, red_color, "sampled side cloud");

    PointCloudTPtr sampled_top_grasps = detector_->getSampledTopGrasps();

    PointCloudColorHandlerCustom<PointT> new_red_color(sampled_top_grasps, 255, 0, 0);
    if (!viewer.updatePointCloud(sampled_top_grasps, new_red_color, "sampled top cloud"))
      viewer.addPointCloud<PointT>(sampled_top_grasps, new_red_color, "sampled top cloud");

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