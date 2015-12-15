//
// Created by yoko on 03-11-15.
//

#include <utils.h>
#include "detection_utils.h"
#include "grasp_point_detector.h"

using namespace moveit_msgs;

namespace bachelors_final_project {

void detection::constructGoal(moveit::planning_interface::MoveGroup &group, PickupGoal &goal_out,
                              const std::string &object, const std::string &planner_name, bool plan_only) {
  goal_out.target_name = object;
  goal_out.group_name = group.getName();
  goal_out.end_effector = group.getEndEffector();
  goal_out.allowed_planning_time = group.getPlanningTime();
  goal_out.support_surface_name = SUPPORT_TABLE();
  goal_out.planner_id = planner_name;
  goal_out.planning_options.plan_only = (unsigned char) plan_only;
  goal_out.planning_options.replan_attempts = 1;

  if (group.getPathConstraints().name != std::string())
    goal_out.path_constraints = group.getPathConstraints();
}


void detection::pickMovement(PickupActionPtr &pick_action_client, PickupGoal &goal, ros::Duration timeout) {
  ROS_FATAL_NAMED(DETECTION(), "\n\n¡¡TRYING REAL PICK ACTION!!\n\n");
  goal.planning_options.plan_only = (unsigned char) false;

  // Try 5 times
  for (int i = 0; i < 5; ++i) {
    pick_action_client->sendGoal(goal);
    if (!pick_action_client->waitForResult(timeout))
      ROS_INFO_STREAM_NAMED(DETECTION(), "Pick action movement returned early");
    actionlib::SimpleClientGoalState state = pick_action_client->getState();

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) break;
    ROS_FATAL_COND_NAMED(i<4, DETECTION(), "PICK ACTION MOVEMENT %s: %s. Trying again...", state.toString().c_str(), state.getText().c_str());

  }

  switch (pick_action_client->getState().state_) {
    case actionlib::SimpleClientGoalState::ABORTED:
      throw ComputeFailedException("MOVEMENT ABORTED. FINISHED EXECUTION");
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      ROS_FATAL_NAMED(DETECTION(), "\n\n¡¡OBJECT REAL PICK ACTION FINISHED!!\n\n");
      break;
    default:
      ROS_FATAL_NAMED(DETECTION(), "\n\nWEIRD THINGS HAPPENNED IN PICK ACTION\n\n");
  }
}

} // namespace bachelors_final_project