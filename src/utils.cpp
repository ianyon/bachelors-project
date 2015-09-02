#include "../include/utils.h"

void utility::publish(const ros::Publisher &pub, const PointCloud<PointXYZ>::Ptr &cloud)
{
    PCLPointCloud2 pointCloudMsg;
    toPCLPointCloud2 (*cloud, pointCloudMsg);
    pub.publish(pointCloudMsg);
}

double utility::durationInMillis(clock_t &begin)
{
    return (double(clock() - begin) / CLOCKS_PER_SEC)*1000;
}
