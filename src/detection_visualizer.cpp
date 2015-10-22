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

const std::string visualization::DetectionVisualizer::OBJ = "object";

const std::string visualization::DetectionVisualizer::ORIGIN_OBJ = "transformed_object";

const std::string visualization::DetectionVisualizer::BOUNDING_BOX = "bounding box";

const std::string visualization::DetectionVisualizer::ORIGIN_BOUNDING_BOX = "transformed bounding box";

const std::string visualization::DetectionVisualizer::EIGEN_VECTOR1 = "eigen1";

const std::string visualization::DetectionVisualizer::EIGEN_VECTOR2 = "eigen2";

const std::string visualization::DetectionVisualizer::SUPPORT_PLANE = "real plane";

const std::string visualization::DetectionVisualizer::SIDE_GRASPS = "sampled side cloud";

const std::string visualization::DetectionVisualizer::TOP_GRASPS = "sampled top cloud";

const std::string visualization::DetectionVisualizer::CENTROID = "centroid";

visualization::DetectionVisualizer::DetectionVisualizer(detection::GraspPointDetector &detector) :
    BaseVisualizer("Detection visualizer"),
    detector_(detector)
{
  configure();
}

void visualization::DetectionVisualizer::configure()
{
  BaseVisualizer::configure();
  std::cout << "Init detection visualize!" << std::endl;
}

void visualization::DetectionVisualizer::computeSpinOnce()
{
  spinOnce(100);
  boost::mutex::scoped_lock bounding_box_lock(detector_.update_bounding_box_mutex_);
  visualizeBoundingBox();
  visualizeSampledGrasps();
  bounding_box_lock.unlock();
}

void visualization::DetectionVisualizer::visualizeBoundingBox()
{
  if (detector_.draw_bounding_box_)
  {
    visualizeCloud(OBJ, detector_.object_cloud_, 180, 180, 180);
    visualizeCloud(ORIGIN_OBJ, detector_.transformed_cloud_, 120, 120, 120);

    // Draw the box
    detection::BoundingBox box = *(detector_.bounding_box_);
    visualizeBox(box, BOUNDING_BOX);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    //transform.translate(bounding_box.translation_);
    transform.translate(box.translation_);
    transform.rotate(box.eigen_vectors_);
    addCoordinateSystem(0.25, transform);

    visualizeBox(box, ORIGIN_BOUNDING_BOX, -0.5f * box.mean_diag_, Eigen::Quaternionf::Identity());

    Point origin(0.0, 0.0, 0.0);
    Point eigen1, eigen2;
    eigen1.getVector3fMap() = box.eigen_vectors_.col(0);
    eigen2.getVector3fMap() = box.eigen_vectors_.col(1);

    visualizeArrow(EIGEN_VECTOR1, origin, eigen1);
    visualizeArrow(EIGEN_VECTOR2, origin, eigen2);

    detector_.draw_bounding_box_ = false;
  }
}

void visualization::DetectionVisualizer::visualizeBox(const detection::BoundingBox &box, const string id)
{
  visualizeBox(box, id, box.translation_, box.rotation_);
}


void visualization::DetectionVisualizer::visualizeBox(const detection::BoundingBox &box, const string id,
                                                      const Eigen::Vector3f &translation,
                                                      const Eigen::Quaternionf &rotation)
{
  removeShape(id);
  addCube(translation, rotation,
          box.max_pt_.x - box.min_pt_.x,
          box.max_pt_.y - box.min_pt_.y,
          box.max_pt_.z - box.min_pt_.z,
          id);
}

void visualization::DetectionVisualizer::visualizeSampledGrasps()
{
  if (detector_.draw_sampled_grasps_)
  {
    Point middle;
    middle.getVector4fMap() = detector_.bounding_box_->centroid_;

    removeShape(SUPPORT_PLANE);
    addPlane(*(detector_.table_plane_), middle.x, middle.y, middle.z, SUPPORT_PLANE);

    PointCloudPtr side_grasps = detector_.getSampledSideGrasps();
    visualizeCloud(SIDE_GRASPS, side_grasps, 255, 0, 0);
    PointCloudPtr top_grasps = detector_.getSampledTopGrasps();
    visualizeCloud(TOP_GRASPS, top_grasps, 255, 0, 0);
    visualizePoint(middle, 0, 0, 255, CENTROID);

    detector_.draw_sampled_grasps_ = false;
  }
}

} // namespace bachelors_final_project
