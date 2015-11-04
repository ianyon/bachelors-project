//
// Created by yoko on 03-11-15.
//

#include <utils.h>
#include "detection_utils.h"
#include "grasp_point_detector.h"

using namespace moveit_msgs;

namespace bachelors_final_project
{

void detection::constructGoal(moveit::planning_interface::MoveGroup &group, PickupGoal &goal_out,
                              const std::string &object, const std::string &planner_name, bool plan_only)
{
  goal_out.target_name = object;
  goal_out.group_name = group.getName();
  goal_out.end_effector = group.getEndEffector();
  goal_out.allowed_planning_time = group.getPlanningTime();
  goal_out.support_surface_name = SUPPORT_TABLE();
  goal_out.planner_id = planner_name;
  goal_out.planning_options.plan_only = (unsigned char) plan_only;
  goal_out.planning_options.replan_attempts = 3;

  if (group.getPathConstraints().name != std::string())
    goal_out.path_constraints = group.getPathConstraints();
}


void detection::pickMovement(PickupActionPtr &pick_action_client, PickupGoal &goal, ros::Duration timeout)
{
  ROS_INFO("TRYING REAL PICK ACTION");
  goal.planning_options.plan_only = (unsigned char) false;

  pick_action_client->sendGoal(goal);
  if (!pick_action_client->waitForResult(timeout))
    ROS_INFO_STREAM_NAMED(DETECTION(), "Pickup action returned early");
  actionlib::SimpleClientGoalState state = pick_action_client->getState();


  while (state.state_ == actionlib::SimpleClientGoalState::ABORTED &&
         selectChoice("Press 1 to retry, any key to finish movement...") == 1)
  {
    pick_action_client->sendGoal(goal);
    if (!pick_action_client->waitForResult(timeout))
      ROS_INFO_STREAM_NAMED(DETECTION(), "Pickup action returned early");
    state = pick_action_client->getState();

    if (state.state_ == actionlib::SimpleClientGoalState::ABORTED)
      ROS_INFO_NAMED(DETECTION(), "Fail: %s: %s.", state.toString().c_str(), state.getText().c_str());
  }

  if (state.state_ == actionlib::SimpleClientGoalState::ABORTED) throw ComputeFailedException("Movement finished.");

  ROS_FATAL_NAMED(DETECTION(), "\n\n¡¡OBJECT PICKED!!\n\n");
}

} // namespace bachelors_final_project