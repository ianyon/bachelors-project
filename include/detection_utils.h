//
// Created by yoko on 03-11-15.
//

#ifndef PROJECT_DETECTION_UTILS_H
#define PROJECT_DETECTION_UTILS_H

#include <string>

#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/PickupGoal.h>

namespace bachelors_final_project
{
namespace detection
{

void constructGoal(moveit::planning_interface::MoveGroup &group, moveit_msgs::PickupGoal &goal_out,
                   const std::string &object, const std::string &planner_name, bool plan_only);

void pickMovement(PickupActionPtr &pick_action_client, moveit_msgs::PickupGoal &goal, ros::Duration timeout);

} // namespace detection
} // namespace bachelors_final_project
#endif //PROJECT_DETECTION_UTILS_H
