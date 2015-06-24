#include <iostream>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>


ros::Publisher pub;

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_publisher");
  ros::NodeHandle nh;

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("input", 1);

  /*** Lógica ***/
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width  = 100;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  // Generate the data
  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    if (rand() % 101 > 30.0)
      cloud.points[i].z = 1.0;
    else // Set a few outliers
      cloud.points[i].z = 128 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Point cloud data: " << cloud.points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " "
                        << cloud.points[i].y << " "
                        << cloud.points[i].z << std::endl;

  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(cloud, pcl_pc2);

  /*** Lógica ***/

  ros::Rate poll_rate(100);
  while(pub.getNumSubscribers() == 0)
      poll_rate.sleep();
  
  // Publish the data.
  pub.publish (pcl_pc2);

  ROS_INFO("%s", "Listo!");
}
