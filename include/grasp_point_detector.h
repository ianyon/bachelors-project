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
static const std::string VARUC = "Detection";

inline std::string const &DETECTION()
{
  static std::string ret = "Detection";
  return ret;
}

inline std::string const &GRASPABLE_OBJECT()
{
  static std::string ret = "graspable object";
  return ret;
}

inline std::string const &SUPPORT_TABLE()
{
  static std::string ret = "table";
  return ret;
}

inline std::string const &LBKPIECE_PLANNER()
{
  static std::string ret = "LBKPIECEkConfigDefault";
  return ret;
}

inline std::string const &RRTCONNECT_PLANNER()
{
  static std::string ret = "RRTConnectkConfigDefault";
  return ret;
}

class GraspPointDetector
{
public:
  GraspPointDetector(ros::NodeHandle &handle, tf::TransformListener &tf_listener);

  bool doProcessing();

  void detect(const CloudPtr &input_object);

  inline CloudPtr getSampledSideGrasps()
  { return sampler.getSideGrasps(); }

  inline CloudPtr getSampledTopGrasps()
  { return sampler.getTopGrasps(); }

  void setTable(const CloudPtr &table_cloud, const pcl::ModelCoefficientsPtr table_plane);

  void setParams(double standoff);

  bool pick(moveit::planning_interface::MoveGroup &group, const std::string &object,
            moveit_msgs::Grasp &grasp, bool plan_only);

  void waitForAction(const PickupActionPtr &action, const ros::Duration &wait_for_server, const std::string &name);

  tf::TransformBroadcaster tf_broadcaster;

  bool draw_bounding_box_, draw_sampled_grasps_;

  boost::mutex update_bounding_box_mutex_;

  ros::NodeHandle &nh_;

  CloudPtr world_obj_, planar_obj_, table_cloud_;

  ros::Publisher pub_cluster_, pub_samples_;

  GraspSampler sampler;
  boost::shared_ptr<GraspFilter> grasp_filter_;
  GraspRanker grasp_ranker_;

  const std::string PLANNER_NAME_;

  BoundingBoxPtr obj_bounding_box_, table_bounding_box_;

  pcl::ModelCoefficientsPtr table_plane_;

  tf::TransformListener &tf_listener_;
  moveit::planning_interface::MoveGroup r_arm_group_;

  PickupActionPtr pick_action_client_;
};

} // namespace detection
} // namespace bachelors_final_project

#endif // GRASP_POINT_DETECTOR_H
