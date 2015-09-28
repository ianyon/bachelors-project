#include "utils.h"

#include <pcl_conversions/pcl_conversions.h>

namespace bachelors_final_project
{

void segmentation::publish(const ros::Publisher &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::PCLPointCloud2 pointCloudMsg;
  pcl::toPCLPointCloud2(*cloud, pointCloudMsg);
  pub.publish(pointCloudMsg);
}

double segmentation::durationMillis(clock_t &begin)
{
  return (double(clock() - begin) / CLOCKS_PER_SEC) * 1000;
}

} // namespace bachelors_final_project