#include "grasp_point_detector.h"

#include <pcl/filters/project_inliers.h>

#include "utils.h"

using namespace pcl;

namespace bachelors_final_project
{

/*
 * Constructor
 */
detection::GraspPointDetector::GraspPointDetector(ros::NodeHandle &handle, tf::TransformListener &tf_listener) :
    grasp_filter_(handle, tf_listener),
    obj_bounding_box_(new BoundingBox("object_base_frame")),
    table_bounding_box_(new BoundingBox("table_base_frame")),
    world_obj_(new Cloud),
    planar_obj_(new Cloud),
    draw_bounding_box_(false),
    draw_sampled_grasps_(false),
    pub_cluster_(handle.advertise<Cloud>("cluster", 1)),
    pub_samples_(handle.advertise<Cloud>("samples", 1)),
    tf_listener_(tf_listener)
{
}  // end GraspPointDetector()

void detection::GraspPointDetector::updateConfig(ParametersConfig &config)
{
  cfg = config;
}

void detection::GraspPointDetector::setTable(const CloudPtr &table_cloud, const pcl::ModelCoefficientsPtr table_plane)
{
  table_plane_ = boost::make_shared<ModelCoefficients>(*table_plane);
  table_cloud_ = table_cloud->makeShared();
}

void detection::GraspPointDetector::detect(const CloudPtr &input_object)
{
  clock_t beginCallback = clock();

  // If no cloud or no new images since last time, do nothing.
  if (input_object->size() == 0) return;

  world_obj_ = input_object->makeShared();
  kinect_frame_id_ = world_obj_->header.frame_id;
  pub_cluster_.publish(world_obj_);

  // Check if computation succeeded
  if (doProcessing())
    ROS_INFO("[%g ms] Detection Success", durationMillis(beginCallback));
  else
    ROS_ERROR("[%g ms] Detection Failed", durationMillis(beginCallback));
}

bool detection::GraspPointDetector::doProcessing()
{
  ROS_INFO_ONCE("'Do Processing' called!");
  clock_t begin = clock();

  /**    COMPUTE BOUNDINGBOX AND PUBLISH A TF FRAME    **/
  boost::mutex::scoped_lock bounding_box_lock(update_bounding_box_mutex_);
  obj_bounding_box_->computeAndPublish(world_obj_, table_plane_, tf_broadcaster);
  planar_obj_ = obj_bounding_box_->getPlanarObj();
  draw_bounding_box_ = true;


  /**    SAMPLING GRASP POSITIONS    **/
  sampler.configure(obj_bounding_box_);
  // Find all the samples positions
  // We call grasp filter to make a pre-filtering step, to avoid generate clearly unfeasible grasps samples
  if (grasp_filter_.generateSideGrasps(obj_bounding_box_, sampler.getSideGraspHeight()))
    sampler.sampleSideGrasps(obj_bounding_box_);
  if (grasp_filter_.generateTopGrasps(obj_bounding_box_, sampler.getTopGraspHeight()))
    sampler.sampleTopGrasps(obj_bounding_box_, grasp_filter_.generateTopGraspsMayorAxis(),
                            grasp_filter_.generateTopGraspsMinorAxis());
  draw_sampled_grasps_ = true;
  bounding_box_lock.unlock();

  CloudPtr side_samples = sampler.getSideGrasps();
  CloudPtr top_samples = sampler.getTopGrasps();
  Cloud samples = *side_samples + *top_samples;
  pub_samples_.publish(samples);
  ROS_INFO("[%g ms] Grasping sampling", durationMillis(begin));


  /**    FILTERING UNFEASIBLE GRASPS    **/
  // We need the table bounding box to create a collision object in moveit
  table_bounding_box_->buildPlanar(table_cloud_);
  try
  {
    begin = clock();
    // Configure filter
    grasp_filter_.configure(sampler.getSideGraspHeight(), sampler.getTopGraspHeight(), obj_bounding_box_,
                            table_bounding_box_);
    // Remove infeasible ones
    grasp_filter_.filterGraspingPoses(side_samples, top_samples);
    ROS_INFO("[%g ms] Grasping filtering", durationMillis(begin));
  }
  catch (ComputeFailedException ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  return true;
}

void detection::GraspPointDetector::setParams(double standoff)
{
  grasp_filter_.setParams(standoff);
}

} // namespace bachelors_final_project
