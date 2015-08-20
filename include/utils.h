#ifndef UTILS
#define UTILS

namespace utility
{
PCLPointCloud2 pointCloudMsg;

//! Publish the message.
void publish(const ros::Publisher pub, const PointCloud<PointXYZ>::Ptr cloud);

//! Compute the time since "begin"
double durationInMillis(clock_t begin);
}

#endif // UTILS

