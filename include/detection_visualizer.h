//
// Created by ian on 9/30/15.
//

#ifndef BACHELORS_FINAL_PROJECT_DETECTIONVISUALIZER_H
#define BACHELORS_FINAL_PROJECT_DETECTIONVISUALIZER_H

#include "base_visualizer.h"

#include <pcl/visualization/pcl_visualizer.h>

#include "containers.h"
#include "bounding_box.h"

namespace bachelors_final_project
{
// Forward declarations
namespace detection
{
class GraspPointDetector;
}

namespace visualization
{
class DetectionVisualizer : public BaseVisualizer
{
  tf::TransformListener &tf_listener_;

public:
  DetectionVisualizer(detection::GraspPointDetector &detector,
                      tf::TransformListener &tf_listener);

  void computeSpinOnce();

  void configure();

  void visualizeBoundingBox();


  void visualizeSampledGrasps();

  detection::GraspPointDetector &obj();

  void visualizeBox(const detection::BoundingBox &box, const std::string id,
                    const Vec3f &translation = Vec3f(0, 0, 0),
                    const Eigen::Quaternionf &rotation = Eigen::Quaternionf::Identity());

  void showEigenVectors(const detection::BoundingBox &box);

  detection::GraspPointDetector &detector_;

  static const std::string OBJ_KINECT_FRAME;
  static const std::string WORLD_PLANAR_OBJ;
  static const std::string OBJ_2D;
  static const std::string OBJ_3D;
  static const std::string WORLD_BOUNDING_BOX;
  static const std::string BOUNDING_BOX;
  static const std::string EIGEN_VECTOR1;
  static const std::string EIGEN_VECTOR2;
  static const std::string SUPPORT_PLANE;
  static const std::string SIDE_GRASPS;
  static const std::string TOP_GRASPS;
  static const std::string CENTROID;
};

} // namespace visualization
} // namespace bachelors_final_project
#endif //BACHELORS_FINAL_PROJECT_DETECTIONVISUALIZER_H
