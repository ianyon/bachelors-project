#ifndef GRASP_POINT_DETECTOR_H
#define GRASP_POINT_DETECTOR_H

#include <stdint.h>

#include <pcl/ModelCoefficients.h>

#include "definitions.h"
#include "containers.h"
#include "grasp_sampler.h"
#include "grasp_filter.h"
#include <bachelors_final_project/ParametersConfig.h>

namespace bachelors_final_project
{
namespace detection
{

class GraspPointDetector
{
public:
  //! Constructor.
  GraspPointDetector(ros::NodeHandle &handle);
  void updateConfig(bachelors_final_project::ParametersConfig &config);

  bool doProcessing();

  void detect(const PointCloudTPtr &cluster, const pcl::ModelCoefficientsPtr &table);

  void computeBoundingBox(PointCloudTPtr &obj_cloud, BoundingBoxPtr&);

  bool draw_bounding_box_;

  PointCloudTPtr object_cloud_;
  PointCloudTPtr transformed_cloud_;

  boost::mutex update_bounding_box_mutex_;

  RankedGrasps ranked_grasps;

  GraspSampler sampler;

  BoundingBoxPtr &bounding_box_;

  bachelors_final_project::ParametersConfig cfg;
  pcl::ModelCoefficientsPtr table_plane_;
  PointCloudTPtr projected_object_;

  bool draw_sampled_grasps_;
  PointCloudTPtr getSampledSideGrasps();

  PointCloudTPtr getSampledTopGrasps();

  GraspFilter grasp_filter_;
  std::string kinect_frame_id_;
};

} // namespace detection
} // namespace bachelors_final_project

#endif // GRASP_POINT_DETECTOR_H
