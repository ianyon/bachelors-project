//
// Created by yoko on 27-10-15.
//

#include "grasp_filter_utils.h"

#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/AttachedCollisionObject.h>

namespace bachelors_final_project
{

geometry_msgs::Pose detection::newPose(Eigen::Vector3f pose)
{
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = pose[0];
  box_pose.position.y = pose[1];
  box_pose.position.z = pose[2];
  return box_pose;
}

geometry_msgs::Pose detection::newPose(Point pose)
{
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = pose.x;
  box_pose.position.y = pose.y;
  box_pose.position.z = pose.z;
  return box_pose;
}

geometry_msgs::Pose detection::newPose(Point pose, Eigen::Quaternionf rotation)
{
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = rotation.w() ;
  box_pose.orientation.x = rotation.x();
  box_pose.orientation.y = rotation.y();
  box_pose.orientation.z = rotation.z();
  box_pose.position.x = pose.x;
  box_pose.position.y = pose.y;
  box_pose.position.z = pose.z;
  return box_pose;
}

moveit_msgs::CollisionObject detection::getCollisionObjUpdated(ros::Publisher pub, const std::string &id,
                                          bool detach, ros::Publisher attached_pub)
{
  moveit_msgs::CollisionObject co;
  co.id = id;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = FOOTPRINT_FRAME;
  // First remove it
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub.publish(co);

  if (detach) // Detach the collision object if attached (for the interest object)
  {
    moveit_msgs::AttachedCollisionObject aco;
    aco.object = co;
    attached_pub.publish(aco);
  }

  co.operation = moveit_msgs::CollisionObject::ADD;
  return co;
}


shape_msgs::SolidPrimitive detection::constructPrimitive(float sizeX, float sizeY, float sizeZ)
{
  shape_msgs::SolidPrimitive primitive;
  primitive.type = shape_msgs::SolidPrimitive::BOX;
  primitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = sizeX;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = sizeY;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = sizeZ;
  return primitive;
}

} // namespace bachelors_final_project