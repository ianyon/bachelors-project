#include <pcl_ros/transforms.h>

#include "utils.h"

#include <pcl/filters/extract_indices.h>


namespace bachelors_final_project
{

void publish(const ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
//  pcl::PCLPointCloud2 pointCloudMsg;
//  pcl::toPCLPointCloud2(*cloud, pointCloudMsg);
  pub.publish(cloud);
}

double durationMillis(clock_t &begin)
{
  return (double(clock() - begin) / CLOCKS_PER_SEC) * 1000;
}

void setProperties(const PointCloudTPtr &coppied_cloud, PointCloudTPtr &cloud_out, int width, int height)
{
  cloud_out->width = (uint32_t) width;
  cloud_out->height = (uint32_t) height;
  // Change the size of the pointcloud
  cloud_out->points.resize((unsigned long) (width * height));
  cloud_out->sensor_origin_ = coppied_cloud->sensor_origin_;
  cloud_out->sensor_orientation_ = coppied_cloud->sensor_orientation_;
  cloud_out->header = coppied_cloud->header;
}

void extractPointCloud(const PointCloudTPtr &input, pcl::PointIndices::Ptr &inliers, PointCloudTPtr &output,
                       bool extract_negative_set)
{
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;

  // Extract the inliers
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(extract_negative_set);
  extract.filter(*output);
  ROS_DEBUG("Extracted PointCloud representing the planar component");
}

void extractPointCloud(const PointCloudNormalPtr &input, pcl::PointIndices::Ptr &inliers, PointCloudNormalPtr &output,
                       bool extract_negative_set)
{
  // Create the filtering object
  pcl::ExtractIndices<Normal> extract;

  // Extract the inliers
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(extract_negative_set);
  extract.filter(*output);
  ROS_DEBUG("Extracted PointCloud representing the planar component");
}

bool transformPoint(const std::string &init_frame, const std::string &final_frame, const PointT &point_in,
                    PointT &point_out, uint64_t micro_sec_time)
{
  tf::TransformListener tf_listener;
  // Constructor requires seconds
  const ros::Time tf_time(micro_sec_time / 1000000);

  try
  {
    ROS_DEBUG("Waiting for transform between %s and %s...", init_frame.c_str(), final_frame.c_str());
    if (not tf_listener.waitForTransform(final_frame, init_frame, tf_time, ros::Duration(TF_TIMEOUT)))
    {
      ROS_ERROR("Couldn't obtain transform from %s to %s in %gs", init_frame.c_str(), final_frame.c_str(), TF_TIMEOUT);
      return false;
    }
    tf::StampedTransform stamped_tf;
    tf_listener.lookupTransform(final_frame, init_frame, tf_time, stamped_tf);

    geometry_msgs::PointStamped point_init_frame;
    point_init_frame.header.frame_id = init_frame;
    point_init_frame.header.stamp = tf_time;
    point_init_frame.header.stamp = stamped_tf.stamp_;
    point_init_frame.point.x = point_in.x;
    point_init_frame.point.y = point_in.y;
    point_init_frame.point.z = point_in.z;

    geometry_msgs::PointStamped point_final_frame;
    tf_listener.transformPoint(final_frame, point_init_frame, point_final_frame);

    ROS_DEBUG("Transformed point [%g,%g,%g] in frame %s to [%g,%g,%g] in frame %s",
              point_in.x, point_in.y, point_in.z, init_frame.c_str(),
              point_final_frame.point.x, point_final_frame.point.y, point_final_frame.point.z,
              final_frame.c_str());

    point_out = PointT((float) point_final_frame.point.x, (float) point_final_frame.point.y,
                  (float) point_final_frame.point.z);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Exception transforming point from frame %s to %s: %s", init_frame.c_str(), final_frame.c_str(),
              ex.what());
    return false;
  }
  return true;
}

bool transformPointCloud(const std::string &init_frame, const std::string &final_frame, const PointCloudTPtr &cloud_in,
                         const PointCloudTPtr &cloud_out, const uint64_t micro_sec_time)
{
  tf::TransformListener tf_listener;
  // Constructor requires seconds
  const ros::Time tf_time(micro_sec_time / 1000000);

  try
  {
    ROS_DEBUG("Waiting for transform between %s and %s...", init_frame.c_str(), final_frame.c_str());
    if (not tf_listener.waitForTransform(final_frame, init_frame, tf_time, ros::Duration(TF_TIMEOUT)))
    {
      ROS_ERROR("Couldn't obtain transform from %s to %s in %gs", init_frame.c_str(), final_frame.c_str(), TF_TIMEOUT);
      return false;
    }

    pcl_ros::transformPointCloud(final_frame, *cloud_in, *cloud_out, tf_listener);

    ROS_DEBUG("Transformed pointcloud of size %lu from frame %s to %s", cloud_out->size(), init_frame.c_str(),
              final_frame.c_str());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Exception transforming pointcloud from frame %s to %s: %s", init_frame.c_str(), final_frame.c_str(),
              ex.what());
    return false;
  }
  return true;
}

} // namespace bachelors_final_project
