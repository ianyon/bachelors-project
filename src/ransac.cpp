#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

int
main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    cloud->points[i].x = rand() % 1024;
    cloud->points[i].y = rand() % 1024;
    if (rand() % 101 > 30.0)
      cloud->points[i].z = 1.0;
    else // Set a few outliers
      cloud->points[i].z = rand() % 128 - 64;
  }

  pcl::visualization::PCLVisualizer viewer("PCL OpenNI Viewer");
  /*                             {170, 96,  57},
                             {37,  112, 89},
                             {45,  136, 45},*/

  viewer.addPointCloud<pcl::PointXYZ>(cloud, "original");

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
  << coefficients->values[1] << " "
  << coefficients->values[2] << " "
  << coefficients->values[3] << std::endl;

  pcl::ExtractIndices<pcl::PointXYZ> extract_;
  extract_.setInputCloud(cloud);
  extract_.setIndices(inliers);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  extract_.filter(*out);


  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(out, 170, 57,  57);
  viewer.addPointCloud<pcl::PointXYZ>(out, color, "table");

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }


  return (0);
}