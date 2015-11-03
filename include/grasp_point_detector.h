#ifndef GRASP_POINT_DETECTOR_H
#define GRASP_POINT_DETECTOR_H

#include <stdint.h>

#include <tf/transform_broadcaster.h>

#include <pcl/ModelCoefficients.h>

#include "definitions.h"
#include "containers.h"
#include "bounding_box.h"
#include "grasp_sampler.h"
#include "grasp_filter.h"
#include "grasp_ranker.h"
#include <bachelors_final_project/ParametersConfig.h>

namespace bachelors_final_project
{
namespace detection
{

class GraspPointDetector
{
public:
  GraspPointDetector(ros::NodeHandle &handle, tf::TransformListener &tf_listener);

  void updateConfig(bachelors_final_project::ParametersConfig &config);

  bool doProcessing();

  void detect(const CloudPtr &input_object);

  inline CloudPtr getSampledSideGrasps()
  { return sampler.getSideGrasps(); }

  inline CloudPtr getSampledTopGrasps()
  { return sampler.getTopGrasps(); }

  void setTable(const CloudPtr &table_cloud, const pcl::ModelCoefficientsPtr table_plane);

  void setParams(double standoff);

  tf::TransformBroadcaster tf_broadcaster;

  bool draw_bounding_box_;
  bool draw_sampled_grasps_;

  boost::mutex update_bounding_box_mutex_;

  std::string kinect_frame_id_;

  CloudPtr world_obj_;

  ros::Publisher pub_cluster_, pub_samples_;

  CloudPtr planar_obj_;

  RankedGrasps ranked_grasps;

  GraspSampler sampler;
  GraspFilter grasp_filter_;
  GraspRanker grasp_ranker_;

  BoundingBoxPtr obj_bounding_box_,table_bounding_box_;

  bachelors_final_project::ParametersConfig cfg;
  CloudPtr table_cloud_;
  pcl::ModelCoefficientsPtr table_plane_;

  tf::TransformListener &tf_listener_;
};

} // namespace detection
} // namespace bachelors_final_project

#endif // GRASP_POINT_DETECTOR_H
