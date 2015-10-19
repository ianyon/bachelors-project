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

  void detect(const PointCloudPtr &cluster, const pcl::ModelCoefficientsPtr &table);

  void computeBoundingBox(PointCloudPtr &obj_cloud, BoundingBoxPtr&);

  PointCloudPtr getSampledSideGrasps();
  PointCloudPtr getSampledTopGrasps();

  bool draw_bounding_box_;
  bool draw_sampled_grasps_;

  boost::mutex update_bounding_box_mutex_;

  std::string kinect_frame_id_;

  PointCloudPtr object_cloud_;
  PointCloudPtr transformed_cloud_;


  RankedGrasps ranked_grasps;

  GraspSampler sampler;
  //GraspFilter grasp_filter_;

  BoundingBoxPtr bounding_box_;

  bachelors_final_project::ParametersConfig cfg;
  pcl::ModelCoefficientsPtr table_plane_;
  PointCloudPtr projected_object_;

};

} // namespace detection
} // namespace bachelors_final_project

#endif // GRASP_POINT_DETECTOR_H
