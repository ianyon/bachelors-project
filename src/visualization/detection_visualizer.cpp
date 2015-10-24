//
// Created by ian on 9/30/15.
//

#include "detection_visualizer.h"
#include "grasp_point_detector.h"

#include <string>
#include <iostream>

#include "bounding_box.h"
#include "utils.h"

using namespace pcl::visualization;
using std::string;

namespace bachelors_final_project
{

const std::string visualization::DetectionVisualizer::WORLD_OBJ = "object";
const std::string visualization::DetectionVisualizer::WORLD_PLANAR_OBJ = "world planar object";
const std::string visualization::DetectionVisualizer::OBJ = "transformed_object";
const std::string visualization::DetectionVisualizer::WORLD_BOUNDING_BOX = "bounding box";
const std::string visualization::DetectionVisualizer::BOUNDING_BOX = "transformed bounding box";
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
  bounding_box_lock.unlock();
  boost::mutex::scoped_lock bounding_box_lock2(detector_.update_bounding_box_mutex_);
  visualizeSampledGrasps();
  bounding_box_lock2.unlock();
}

void visualization::DetectionVisualizer::visualizeBoundingBox()
{
  if (detector_.draw_bounding_box_)
  {
    detection::BoundingBox box = *(detector_.bounding_box_);

    // Draw world coordinate system
    addCoordinateSystem(0.25, box.getObjectToWorldTransform());

    visualizeCloud(WORLD_OBJ, detector_.world_obj_, 180, 180, 180);
    visualizeCloud(OBJ, detector_.planar_obj_, 120, 120, 120);
    visualizeCloud(WORLD_PLANAR_OBJ, detector_.world_planar_obj_, 180, 180, 180);

    // Draw the box in world coords
    visualizeBox(box, WORLD_BOUNDING_BOX, box.obj_to_world_translation_, box.obj_to_world_rotation_);
    // Draw the box in the origin (obj coords)
    visualizeBox(box, BOUNDING_BOX);

    //showEigenVectors(box);

    detector_.draw_bounding_box_ = false;
  }
}

void visualization::DetectionVisualizer::showEigenVectors(const detection::BoundingBox &box)
{
  bachelors_final_project::Point eigen1, eigen2;
  eigen1.getVector3fMap() = box.eigen_vectors_.col(0);
  eigen2.getVector3fMap() = box.eigen_vectors_.col(1);
  visualizeArrow(bachelors_final_project::visualization::DetectionVisualizer::EIGEN_VECTOR1, eigen1);
  visualizeArrow(bachelors_final_project::visualization::DetectionVisualizer::EIGEN_VECTOR2, eigen2);
}

/**
 * Draw a bounding box centered at (0,0,0) and translate and rotate it accordingly
 */
void visualization::DetectionVisualizer::visualizeBox(const detection::BoundingBox &box, const string id,
                                                      const Eigen::Vector3f &translation,
                                                      const Eigen::Quaternionf &rotation)
{
  removeShape(id);
  addCube(translation, rotation,
          box.max_pt_planar_centroid_.x - box.min_pt_planar_centroid_.x,
          box.max_pt_planar_centroid_.y - box.min_pt_planar_centroid_.y,
          box.max_pt_planar_centroid_.z - box.min_pt_planar_centroid_.z,
          id);
}

void visualization::DetectionVisualizer::visualizeSampledGrasps()
{
  if (detector_.draw_sampled_grasps_)
  {
    Point middle;
    middle.getVector4fMap() = detector_.bounding_box_->world_coords_planar_centroid_;

    removeShape(SUPPORT_PLANE);
    addPlane(*(detector_.table_plane_), middle.x, middle.y, middle.z, SUPPORT_PLANE);

    CloudPtr side_grasps = detector_.getSampledSideGrasps();
    visualizeCloud(SIDE_GRASPS, side_grasps, 255, 0, 0);
    CloudPtr top_grasps = detector_.getSampledTopGrasps();
    visualizeCloud(TOP_GRASPS, top_grasps, 255, 0, 0);
    visualizePoint(middle, 0, 0, 255, CENTROID);

    detector_.draw_sampled_grasps_ = false;
  }
}

} // namespace bachelors_final_project
