#ifndef UTILS
#define UTILS

#include "definitions.h"

#include <ros/ros.h>

namespace bachelors_final_project
{
//! Publish the message.
void publish(const ros::Publisher &, const PointCloudTPtr&);

//! Compute the time since "begin"
double durationMillis(clock_t &);

} // namespace bachelors_final_project

#endif // UTILS

