#ifndef GRASP_POINT_DETECTOR_H
#define GRASP_POINT_DETECTOR_H

#include <stdint.h>

#include "definitions.h"
#include <bachelors_final_project/ParametersConfig.h>
#include <pcl/ModelCoefficients.h>

namespace bachelors_final_project
{
namespace detection
{

class GraspPointDetector
{
public:
  //! Constructor.
  GraspPointDetector();
  void updateConfig(bachelors_final_project::ParametersConfig &config);

  bool doProcessing();

  void detect(const PointCloudTPtr &, const pcl::ModelCoefficientsPtr &);

  void boundingBox(PointCloudTPtr &cloud, BoundingBox*);

  bool draw_bounding_box_;

  PointCloudTPtr object_cloud_;
  PointCloudTPtr transformed_cloud_;

  struct BoundingBox
  {
    void initialize(PointT _min_pt, PointT _max_pt, Eigen::Quaternionf _rotation, Eigen::Vector3f _translation,
      Eigen::Vector4f _centroid, Eigen::Matrix3f _eigen_vectors, Eigen::Vector3f _mean_diag)
    {
      min_pt = _min_pt;
      max_pt = _max_pt;
      rotation = _rotation;
      translation = _translation;
      centroid = _centroid;
      eigen_vectors = _eigen_vectors;
      mean_diag = _mean_diag;
    }

    PointT min_pt;
    PointT max_pt;
    // rotation represents the eigen vectors and a rotation matrix
    Eigen::Quaternionf rotation;
    Eigen::Vector3f translation;
    Eigen::Vector4f centroid;
    Eigen::Matrix3f eigen_vectors;
    Eigen::Vector3f mean_diag;
  };

  struct GraspTypesContainer
  {
  	std::vector<PointT> side_grasps, top_grasps;
  };

  struct RankedGrasps
  {
  	std::vector<PointT> side_grasps, top_grasps;
  };

  GraspTypesContainer sampled_grasps_;
  GraspTypesContainer feasible_grasps;
  RankedGrasps ranked_grasps;

  BoundingBox bounding_box_;

  bachelors_final_project::ParametersConfig cfg;

  pcl::ModelCoefficientsPtr table_plane_;
  PointCloudTPtr projected_object_;
};

} // namespace detection
} // namespace bachelors_final_project

#endif // GRASP_POINT_DETECTOR_H
