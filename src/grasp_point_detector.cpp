#include "grasp_point_detector.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <bounding_box.h>
#include <tf/transform_broadcaster.h>

#include "utils.h"

using namespace pcl;

namespace bachelors_final_project
{

/*
 * Constructor
 */
detection::GraspPointDetector::GraspPointDetector(ros::NodeHandle &handle, tf::TransformListener &tf_listener) :
    grasp_filter_(handle, tf_listener),
    obj_bounding_box_(new BoundingBox("object_frame")),
    table_bounding_box_(new BoundingBox("table_frame")),
    world_obj_(new Cloud),
    planar_obj_(new Cloud),
    world_planar_obj_(new Cloud),
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

  // Project to table to obtain object projection
  projectOnPlane(world_obj_, table_plane_, world_planar_obj_);
  // world_planar_obj_ lies in plane YZ with a = blue (z) - b = green (y)

  clock_t begin = clock();
  boost::mutex::scoped_lock bounding_box_lock(update_bounding_box_mutex_);

  obj_bounding_box_->buildPlanar(world_planar_obj_);
  obj_bounding_box_->build3DAndPublishFrame(world_obj_, tf_broadcaster);
  planar_obj_ = obj_bounding_box_->getPlanarObj();
  draw_bounding_box_ = true;

  // Find all the samples poses
  sampler.sampleGraspingPoses(obj_bounding_box_, kinect_frame_id_);
  draw_sampled_grasps_ = true;
  Cloud samples = *sampler.getSideGrasps();
  samples += *sampler.getTopGrasps();
  pub_samples_.publish(samples);
  bounding_box_lock.unlock();
  ROS_INFO("[%g ms] Grasping sampling", durationMillis(begin));

  // We need the table bounding box to create a collision object in moveit
  table_bounding_box_->buildPlanar(table_cloud_);
  table_bounding_box_->broadcast2DFrameUpdate(tf_broadcaster);
  try
  {
    begin = clock();
    // Configure filter
    grasp_filter_.configure(kinect_frame_id_, obj_bounding_box_, table_bounding_box_);
    // Remove infeasible ones
    grasp_filter_.filterGraspingPoses(sampler.getSideGrasps(), sampler.getTopGrasps());
    ROS_INFO("[%g ms] Grasping filtering", durationMillis(begin));
  }
  catch (ComputeFailedException ex)
  {
    return false;
  }

  return true;
}


} // namespace bachelors_final_project
