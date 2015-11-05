#include "grasp_point_detector.h"

#include <pcl/filters/project_inliers.h>

#include <moveit/move_group_pick_place_capability/capability_names.h>

#include "utils.h"
#include "definitions.h"
#include "detection_utils.h"

using namespace pcl;

namespace bachelors_final_project
{

/*
 * Constructor
 */
detection::GraspPointDetector::GraspPointDetector(ros::NodeHandle &handle, tf::TransformListener &tf_listener) :
    r_arm_group_("right_arm"),
    grasp_ranker_(tf_listener),
    obj_bounding_box_(new BoundingBox("object_base_frame")),
    table_bounding_box_(new BoundingBox("table_base_frame")),
    world_obj_(new Cloud),
    planar_obj_(new Cloud),
    draw_bounding_box_(false),
    draw_sampled_grasps_(false),
    pub_cluster_(handle.advertise<Cloud>("cluster", 1)),
    pub_samples_(handle.advertise<Cloud>("samples", 1)),
    tf_listener_(tf_listener),
    pick_action_client_(new PickupAction(move_group::PICKUP_ACTION, false)),
    nh_(handle),
    PLANNER_NAME_(RRTCONNECT_PLANNER())
{
  grasp_filter_.reset(new GraspFilter(handle, tf_listener, r_arm_group_, pick_action_client_, PLANNER_NAME_));
  waitForAction(pick_action_client_, ros::Duration(5.0), move_group::PICKUP_ACTION);
}  // end GraspPointDetector()

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
  geometry_msgs::PoseStamped initial_pose = r_arm_group_.getCurrentPose(r_arm_group_.getEndEffectorLink());

  /**    COMPUTE BOUNDINGBOX AND PUBLISH A TF FRAME    **/
  boost::mutex::scoped_lock bounding_box_lock(update_bounding_box_mutex_);
  obj_bounding_box_->computeAndPublish(world_obj_, table_plane_, tf_broadcaster);
  planar_obj_ = obj_bounding_box_->getPlanarObj();
  draw_bounding_box_ = true;


  /**    SAMPLING GRASP POSITIONS    **/
  sampler.configure(obj_bounding_box_);
  // Find all the samples positions
  // We call grasp filter to make a pre-filtering step, to avoid generate clearly unfeasible grasps samples
  if(!sampler.generateSamples(obj_bounding_box_, &GraspFilter::generateSideSamples, &GraspFilter::generateTopSamples))
  {
    bounding_box_lock.unlock();
    return false;
  }
  draw_sampled_grasps_ = true;
  bounding_box_lock.unlock();
  pub_samples_.publish(sampler.getSamples());
  ROS_INFO("[%g ms] Grasping sampling", durationMillis(begin));


  /**    FILTERING UNFEASIBLE GRASPS    **/
  // We need the table bounding box to create a collision object in moveit
  table_bounding_box_->buildPlanar(table_cloud_);
  try
  {
    begin = clock();
    // Configure filter
    grasp_filter_->configure(sampler.getSideGraspHeight(), sampler.getTopGraspHeight(), obj_bounding_box_,
                             table_bounding_box_);
    // Remove infeasible ones
    grasp_filter_->filterGraspingPoses(sampler.getSideGrasps(), sampler.getTopGrasps());
    ROS_INFO("[%g ms] Grasping filtering", durationMillis(begin));
  }
  catch (ComputeFailedException ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  if (!grasp_filter_->feasibleGrasps()) return false;

  /**    GRASP RANKING    **/
  begin = clock();
  grasp_ranker_.configure(obj_bounding_box_, grasp_filter_->getEndEffectorLink(), grasp_filter_->getShoulderLink());
  grasp_ranker_.rankGraspingPoses(grasp_filter_->getSideGrasps(), grasp_filter_->getTopGrasps());
  moveit_msgs::Grasp grasp_winner = grasp_ranker_.getSelectedGrasp();
  ROS_INFO("[%g ms] Grasping ranking", durationMillis(begin));

  // Show winner plan
  pick(r_arm_group_, GRASPABLE_OBJECT(), grasp_winner, true);

  if (selectChoice("Ready: 1=Pick, Any=Again") != 1) return false;

  /**    ACTUAL PICKING    **/
  begin = clock();
  r_arm_group_.setPlanningTime(25.0);
  r_arm_group_.setSupportSurfaceName(SUPPORT_TABLE());
  bool result = pick(r_arm_group_, GRASPABLE_OBJECT(), grasp_winner, false);
  ROS_INFO("[%g ms] Pick movement", durationMillis(begin));

  r_arm_group_.setPoseTarget(initial_pose, r_arm_group_.getEndEffectorLink());
  moveit::planning_interface::MoveGroup::Plan plan;
  bool success = r_arm_group_.plan(plan);
  if (success) r_arm_group_.move();

  return result;
}

void detection::GraspPointDetector::setParams(double standoff)
{
  grasp_filter_->setParams(standoff);
}

bool detection::GraspPointDetector::pick(moveit::planning_interface::MoveGroup &group, const std::string &object,
                                         moveit_msgs::Grasp &grasp, bool plan_only)
{
  if (!pick_action_client_)
  {
    ROS_ERROR_STREAM("Pick action client not found");
    return false;
  }
  if (!pick_action_client_->isServerConnected())
  {
    ROS_ERROR_STREAM("Pick action server not connected");
    return false;
  }

  moveit_msgs::PickupGoal goal;
  constructGoal(group, goal, object, PLANNER_NAME_, plan_only);
  goal.possible_grasps.push_back(grasp);
  if (plan_only)
  {
    pick_action_client_->cancelAllGoals();
    pick_action_client_->sendGoal(goal);
    if (!pick_action_client_->waitForResult(ros::Duration(2.0)))
      ROS_DEBUG_STREAM("Pickup action returned early");

    bool plan_succeeded = pick_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    ROS_DEBUG_STREAM_COND(!plan_succeeded, "Fail: " << pick_action_client_->getState().toString() << ": " <<
                                           pick_action_client_->getState().getText());
  }
  else
  {
    try
    {
      pickMovement(pick_action_client_, goal, ros::Duration(10.0));
    }
    catch (ComputeFailedException ex)
    {
      ROS_INFO_NAMED(DETECTION(), "%s", ex.what());
      return false;
    }
  }
  return true;
}

void detection::GraspPointDetector::waitForAction(const PickupActionPtr &action, const ros::Duration &wait_for_server,
                                                  const std::string &name)
{
  ROS_DEBUG_NAMED(DETECTION(), "Waiting for MoveGroup action server (%s)...", name.c_str());

  // in case ROS time is published, wait for the time data to arrive
  ros::Time start_time = ros::Time::now();
  while (start_time == ros::Time::now())
  {
    ros::WallDuration(0.01).sleep();
    ros::spinOnce();
  }

  // wait for the server (and spin as needed)
  if (wait_for_server == ros::Duration(0, 0))
  {
    while (nh_.ok() && !action->isServerConnected())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }
  else
  {
    ros::Time final_time = ros::Time::now() + wait_for_server;
    while (nh_.ok() && !action->isServerConnected() && final_time > ros::Time::now())
    {
      ros::WallDuration(0.02).sleep();
      ros::spinOnce();
    }
  }

  if (!action->isServerConnected())
    throw std::runtime_error("Unable to connect to action server within allotted time");
  else
    ROS_DEBUG_NAMED(DETECTION(), "Connected to '%s'", name.c_str());
}

} // namespace bachelors_final_project
