#ifndef GRASP_POINT_DETECTOR_H
#define GRASP_POINT_DETECTOR_H

#include <stdint.h>

#include <pcl/ModelCoefficients.h>

#include "definitions.h"
#include "containers.h"
#include "grasp_sampler.h"
#include <bachelors_final_project/ParametersConfig.h>

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

  void computeBoundingBox(PointCloudTPtr &obj_cloud, BoundingBox *);

  bool draw_bounding_box_;

  PointCloudTPtr object_cloud_;
  PointCloudTPtr transformed_cloud_;

  boost::mutex update_bounding_box_mutex_;

  GraspTypesContainer feasible_grasps;

  RankedGrasps ranked_grasps;

  GraspSampler sampler;

  BoundingBox bounding_box_;

  bachelors_final_project::ParametersConfig cfg;
  pcl::ModelCoefficientsPtr table_plane_;
  PointCloudTPtr projected_object_;

  bool draw_sampled_grasps_;
  PointCloudTPtr getSampledSideGrasps();

  PointCloudTPtr getSampledTopGrasps();
};

} // namespace detection
} // namespace bachelors_final_project

#endif // GRASP_POINT_DETECTOR_H
