#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

int
main(int argc, char **argv)
{

  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->header.frame_id = "some_tf_frame";
  cloud->height = 32;
  cloud->width = 32;
  cloud->is_dense = true;
  cloud->points.resize(32 * 32);

  for (int u = 0; u < 32; u++)
  {
    for (int v = 0; v < 32; v++)
    {
      pcl::PointXYZ result;
      if (rand() % 101 > 30.0)
        result.z = 10.0;
      else // Set a few outliers
        result.z = rand() % 128 - 54;
      result.x = u * (1024 / 32);
      result.y = v * (1024 / 32);
      cloud->points[u * 32 + v] = result;
    }
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud, 0, 255, 57);
  viewer.addPointCloud<pcl::PointXYZ>(cloud, color1, "original");

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(100.0);
  ne.compute(*cloud_normals);
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 1, 100.0, "NORMALS_CLOUD");


  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sac_segmentation_;
  sac_segmentation_.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
  sac_segmentation_.setMethodType(pcl::SAC_RANSAC);
  sac_segmentation_.setNormalDistanceWeight(0.2);
  sac_segmentation_.setDistanceThreshold(10.0);
  sac_segmentation_.setMaxIterations(200);
  sac_segmentation_.setOptimizeCoefficients(true);
  sac_segmentation_.setInputCloud(cloud);
  sac_segmentation_.setInputNormals(cloud_normals);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  sac_segmentation_.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract_;
  extract_.setInputCloud(cloud);
  extract_.setIndices(inliers);
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  extract_.filter(*out);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(out, 170, 57, 57);
  viewer.addPointCloud<pcl::PointXYZ>(out, color, "table");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }


  return (0);
}