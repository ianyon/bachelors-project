#include "../include/utils.h"

#include <pcl_conversions/pcl_conversions.h>

void utils::publish(const ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::PCLPointCloud2 pointCloudMsg;
  pcl::toPCLPointCloud2(*cloud, pointCloudMsg);
  pub.publish(pointCloudMsg);
}

double utils::durationInMillis(clock_t &begin)
{
  return (double(clock() - begin) / CLOCKS_PER_SEC) * 1000;
}