#include <pcl_ros/transforms.h>

#include "utils.h"

#include <pcl/filters/extract_indices.h>

using boost::str;
using boost::format;


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

void setProperties(const PointCloudPtr &coppied_cloud, PointCloudPtr &cloud_out, int width, int height)
{
  cloud_out->width = (uint32_t) width;
  cloud_out->height = (uint32_t) height;
  // Change the size of the pointcloud
  cloud_out->points.resize((unsigned long) (width * height));
  cloud_out->sensor_origin_ = coppied_cloud->sensor_origin_;
  cloud_out->sensor_orientation_ = coppied_cloud->sensor_orientation_;
  cloud_out->header = coppied_cloud->header;
}

void pointCloudFromIndices(const PointCloudPtr &input, pcl::PointIndicesPtr &inliers, PointCloudPtr &output,
                           bool extract_negative_set)
{
  // Create the filtering object
  pcl::ExtractIndices<Point> extract;

  // Extract the inliers
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(extract_negative_set);
  extract.filter(*output);
}

void normalPointCloudFromIndices(const PointCloudNormalPtr &input, pcl::PointIndicesPtr &inliers,
                                 PointCloudNormalPtr &output,
                                 bool extract_negative_set)
{
  // Create the filtering object
  pcl::ExtractIndices<Normal> extract;

  // Extract the inliers
  extract.setInputCloud(input);
  extract.setIndices(inliers);
  extract.setNegative(extract_negative_set);
  extract.filter(*output);
}

bool transformPoint(const std::string &init_frame, const std::string &final_frame, const tf::Vector3 &point_in,
                    Point &point_out, uint64_t micro_sec_time, tf::TransformListener &tf_listener)
{
  // Constructor requires seconds
  ros::Time tf_time(micro_sec_time / 1000000.0);
  try
  {
    if (not tf_listener.waitForTransform(final_frame, init_frame, tf_time, TF_DURATION))
      throw tf::TransformException(str( format("Timeout [%gs]") % TF_TIMEOUT));

    tf::StampedTransform stamped_tf;
    tf_listener.lookupTransform(final_frame, init_frame, tf_time, stamped_tf);
    tf::Vector3 point_final_frame = stamped_tf*point_in;

    ROS_DEBUG("TF %s to %s [%g,%g,%g]", init_frame.c_str(), final_frame.c_str(),
              point_final_frame.x(), point_final_frame.y(), point_final_frame.z());

    point_out = Point((float) point_final_frame.x(), (float) point_final_frame.y(), (float) point_final_frame.z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Exception transforming point from frame %s to %s: %s", init_frame.c_str(), final_frame.c_str(),
              ex.what());
    return false;
  }
  return true;
}

bool transformPointCloud(const std::string &init_frame, const std::string &final_frame, const PointCloudPtr &cloud_in,
                         const PointCloudPtr &cloud_out, const uint64_t micro_sec_time,
                         tf::TransformListener &tf_listener)
{
  // Constructor requires seconds
  ros::Time tf_time(micro_sec_time / 1000000.0);

  try
  {
    if (not tf_listener.waitForTransform(final_frame, init_frame, tf_time, TF_DURATION))
      throw tf::TransformException(str( format("Timeout [%gs]") % TF_TIMEOUT));

    pcl_ros::transformPointCloud(final_frame, tf_time, *cloud_in, init_frame, *cloud_out, tf_listener);

    ROS_DEBUG("Transform pointcloud [%lu] from %s to %s", cloud_out->size(), init_frame.c_str(), final_frame.c_str());
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
