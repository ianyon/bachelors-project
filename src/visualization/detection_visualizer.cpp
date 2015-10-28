//
// Created by ian on 9/30/15.
//

#include "detection_visualizer.h"
#include "grasp_point_detector.h"

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
  setCameraPosition(0.414395, -0.134601, 0.669816, 0.190149, 0.0424081, 1.09322, -0.13325, -0.93753, 0.321374);
  setCameraFieldOfView(0.8575);
  setCameraClipDistances(0.0067374, 6.7374);
  setPosition(637, 145);
  setSize(727, 619);
  ROS_INFO("Initiated detection visualizer!");
}

void visualization::DetectionVisualizer::computeSpinOnce()
{
  spinOnce(100);
  boost::mutex::scoped_lock bounding_box_lock(obj().update_bounding_box_mutex_);
  visualizeBoundingBox();
  visualizeSampledGrasps();
  bounding_box_lock.unlock();
}

void visualization::DetectionVisualizer::visualizeBoundingBox()
{
  if (obj().draw_bounding_box_)
  {
    detection::BoundingBox box = *(obj().obj_bounding_box_);

    // Draw world coordinate system
    removeCoordinateSystem();
    addCoordinateSystem(0.5);
    addCoordinateSystem(0.25, box.getObjectToWorldTransform());

    visualizeCloud(WORLD_OBJ, obj().world_obj_, 180, 180, 180);
    visualizeCloud(OBJ, obj().planar_obj_, 120, 120, 120);
    visualizeCloud(WORLD_PLANAR_OBJ, obj().world_planar_obj_, 180, 180, 180);

    // Draw the box in world coords
    visualizeBox(box, WORLD_BOUNDING_BOX, box.position_2D_kinect_frame_, box.rotation_kinect_frame_);
    // Draw the box in the origin (obj coords)
    visualizeBox(box, BOUNDING_BOX);

    //showEigenVectors(box);

    obj().draw_bounding_box_ = false;
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
  if (obj().draw_sampled_grasps_)
  {
    Point middle;
    middle.getVector4fMap() = obj().obj_bounding_box_->world_coords_planar_centroid_;

    removeShape(SUPPORT_PLANE);
    addPlane(*(obj().table_plane_), middle.x, middle.y, middle.z, SUPPORT_PLANE);

    CloudPtr side_grasps = obj().getSampledSideGrasps();
    visualizeCloud(SIDE_GRASPS, side_grasps, 255, 0, 0);
    CloudPtr top_grasps = obj().getSampledTopGrasps();
    visualizeCloud(TOP_GRASPS, top_grasps, 255, 0, 0);
    visualizePoint(middle, 0, 0, 255, CENTROID);

    obj().draw_sampled_grasps_ = false;
  }
}

detection::GraspPointDetector& visualization::DetectionVisualizer::obj()
{
  return detector_;
}

} // namespace bachelors_final_project
