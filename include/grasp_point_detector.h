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

  void boundingBox(PointCloudTPtr &);

  bool draw_bounding_box_;

  PointCloudTPtr object_cloud_;
  PointCloudTPtr transformed_cloud_;

  struct BoundingBoxParameters
  {
    PointT min_pt, max_pt;
    Eigen::Quaternionf q_final;
    Eigen::Vector3f t_final;
  } bounding_box_parameters_;

  bachelors_final_project::ParametersConfig cfg;

  pcl::ModelCoefficientsPtr table_plane_;
  PointCloudTPtr projected_object_;
};

} // namespace detection
} // namespace bachelors_final_project

#endif // GRASP_POINT_DETECTOR_H