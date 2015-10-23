#include "grasp_point_detector.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <bounding_box.h>

#include "utils.h"

using namespace pcl;

namespace bachelors_final_project
{

/*
 * Constructor
 */
detection::GraspPointDetector::GraspPointDetector(ros::NodeHandle &handle)
{
  bounding_box_.reset(new BoundingBox);
  //grasp_filter_.initializePublisher(handle);

  // Initialize pointers to point clouds
  object_cloud_.reset(new Cloud);
  transformed_cloud_.reset(new Cloud);
  projected_object_.reset(new Cloud);

  draw_bounding_box_ = false;
  draw_sampled_grasps_ = false;

}  // end GraspPointDetector()

void detection::GraspPointDetector::updateConfig(ParametersConfig &config)
{
  cfg = config;
}

void detection::GraspPointDetector::detect(const CloudPtr &input_object, const pcl::ModelCoefficientsPtr &table_plane)
{
  clock_t beginCallback = clock();

  // If no cloud or no new images since last time, do nothing.
  if (input_object->size() == 0) return;

  object_cloud_ = input_object->makeShared();
  table_plane_ = boost::make_shared<ModelCoefficients>(*table_plane);

  kinect_frame_id_ = object_cloud_->header.frame_id;

  // Check if computation succeeded
  if (doProcessing())
    ROS_INFO("Detection callback took %gms\n\n", durationMillis(beginCallback));
}

bool detection::GraspPointDetector::doProcessing()
{
  ROS_INFO_ONCE("'Do Processing' called!");

  // Project to table to obtain object projection
  projectOnPlane(object_cloud_, table_plane_, projected_object_);
  // projected_object_ lies in plane YZ with a = blue (z) - b = green (y)

  clock_t begin = clock();
  boost::mutex::scoped_lock bounding_box_lock(update_bounding_box_mutex_);
  bounding_box_ = BoundingBoxPtr(new BoundingBox());
  bounding_box_->build(projected_object_, object_cloud_, transformed_cloud_);
  draw_bounding_box_ = true;

  // Find all the samples poses
  sampler.sampleGraspingPoses(bounding_box_);
  draw_sampled_grasps_ = true;
  bounding_box_lock.unlock();
  ROS_INFO("Grasping sampling  took %gms", durationMillis(begin));

  begin = clock();
  // Configure filter
  //grasp_filter_.configure(kinect_frame_id_, bounding_box_, table_plane_);
  // Remove infeasible ones
  //grasp_filter_.filterGraspingPoses(sampler.getSideGrasps(), sampler.getTopGrasps());
  //ROS_INFO("Grasping filtering took %gms", durationMillis(begin));

  return true;
}


} // namespace bachelors_final_project
