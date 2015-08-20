#include "utility.hpp"

namespace utility
{

void publish(const ros::Publisher pub, const PointCloud<PointXYZ>::Ptr cloud)
{
    toPCLPointCloud2 (*cloud, pointCloudMsg);
    pub.publish(pointCloudMsg);
}

double durationInMillis(clock_t begin)
{
    return (double(clock() - begin) / CLOCKS_PER_SEC)*1000;
}

}
