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

const std::string visualization::DetectionVisualizer::OBJ_KINECT_FRAME = "object";
const std::string visualization::DetectionVisualizer::WORLD_PLANAR_OBJ = "world planar object";
const std::string visualization::DetectionVisualizer::OBJ_2D = "transformed_object";
const std::string visualization::DetectionVisualizer::OBJ_3D = "transformed_object 3d";
const std::string visualization::DetectionVisualizer::WORLD_BOUNDING_BOX = "bounding box";
const std::string visualization::DetectionVisualizer::BOUNDING_BOX = "transformed bounding box";
const std::string visualization::DetectionVisualizer::EIGEN_VECTOR1 = "eigen1";
const std::string visualization::DetectionVisualizer::EIGEN_VECTOR2 = "eigen2";
const std::string visualization::DetectionVisualizer::SUPPORT_PLANE = "real plane";
const std::string visualization::DetectionVisualizer::SIDE_GRASPS = "sampled side cloud";
const std::string visualization::DetectionVisualizer::TOP_GRASPS = "sampled top cloud";
const std::string visualization::DetectionVisualizer::CENTROID = "centroid";

visualization::DetectionVisualizer::DetectionVisualizer(detection::GraspPointDetector &detector,
                                                        tf::TransformListener &tf_listener) :
    BaseVisualizer("Detection visualizer"),
    detector_(detector),
    tf_listener_(tf_listener)
{
  configure();
}

void visualization::DetectionVisualizer::configure()
{
  BaseVisualizer::configure();
  setCameraPosition(0.214395, 0.134601, 0.369816, 0.190149, 0.0424081, 1.09322, -0.13325, -0.93753, 0.321374);
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
    detection::BoundingBox &box = *(obj().obj_bounding_box_);

    // Draw world coordinate system
    removeCoordinateSystem();
    addCoordinateSystem(0.5);
    addCoordinateSystem(0.25, box.getObjToKinectTransform());

    CloudPtr &obj_kinect_frame = obj().world_obj_;
    visualizeCloud(OBJ_KINECT_FRAME, obj_kinect_frame, 0, 255, 0);
    visualizeCloud(OBJ_2D, obj().planar_obj_, 120, 120, 120);
    CloudPtr obj_frame(new Cloud);
    transformPointCloud(obj_kinect_frame->header.frame_id, box.OBJ_FRAME, obj_kinect_frame, obj_frame,
                        obj_kinect_frame->header.stamp, tf_listener_);
    visualizeCloud(OBJ_3D, obj_frame, 0, 0, 255);
    visualizeCloud(WORLD_PLANAR_OBJ, box.obj_2D_kinect_frame, 0, 0, 255);

    // Draw the box in world coords
    visualizeBox(box, WORLD_BOUNDING_BOX, box.position_3D_kinect_frame_, box.getRotationQuaternion());
    // Draw the box in the origin (obj coords)
    visualizeBox(box, BOUNDING_BOX);

    CloudPtr a(new Cloud);
    a->push_back(newPoint(box.position_base_kinect_frame_));
    a->push_back(newPoint(box.position_3D_kinect_frame_));
    visualizeCloud("a", a, 0, 0, 255);
    CloudPtr b(new Cloud);
    b->push_back(newPoint(box.centroid_2D_kinect_frame_.head<3>()));
    b->push_back(newPoint(box.planar_shift_));
    //b->push_back(newPoint(box.));
    visualizeCloud("b", b, 255, 0, 0);

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
                                                      const Vec3f &translation,
                                                      const Eigen::Quaternionf &rotation)
{
  removeShape(id);
  addCube(translation, rotation,
          box.getSize3D()[0],
          box.getSize3D()[1],
          box.getSize3D()[2],
          id);
}

void visualization::DetectionVisualizer::visualizeSampledGrasps()
{
  if (obj().draw_sampled_grasps_)
  {
    Point middle;
    middle.getVector3fMap() = obj().obj_bounding_box_->position_base_kinect_frame_;

    removeShape(SUPPORT_PLANE);
    addPlane(*(obj().table_plane_), middle.x, middle.y, middle.z, SUPPORT_PLANE);

    CloudPtr side_grasps = obj().getSampledSideGrasps();
    const pcl::PCLHeader &header = obj().world_obj_->header;
    transformPointCloud(FOOTPRINT_FRAME, header.frame_id, side_grasps, side_grasps,header.stamp,tf_listener_);
    visualizeCloud(SIDE_GRASPS, side_grasps, 255, 0, 0);
    CloudPtr top_grasps = obj().getSampledTopGrasps();
    transformPointCloud(FOOTPRINT_FRAME, header.frame_id, top_grasps, top_grasps,header.stamp,tf_listener_);
    visualizeCloud(TOP_GRASPS, top_grasps, 255, 0, 0);
    //visualizePoint(middle, 0, 0, 255, CENTROID);

    obj().draw_sampled_grasps_ = false;
  }
}

detection::GraspPointDetector &visualization::DetectionVisualizer::obj()
{
  return detector_;
}

} // namespace bachelors_final_project
