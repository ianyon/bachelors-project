#include "segmentation_visualizer.h"

#include <iostream>

#include "definitions.h"
#include "cloud_segmentator.h"

using namespace pcl::visualization;

namespace bachelors_final_project
{

visualization::SegmentationVisualizer::SegmentationVisualizer(segmentation::CloudSegmentator *segmentator) :
    segmentator_(segmentator)
{
  std::cout << "INIT constructor!" << std::endl;
  const int colors[10][3] = {{170, 57,  57},
                             {170, 96,  57},
                             {37,  112, 89},
                             {45,  136, 45},
                             {128, 43,  102},
                             {170, 155, 57},
                             {69,  47,  116},
                             {132, 161, 54},
                             {137, 162, 54},
                             {170, 123, 57}};
  *(array_t *) colors_ = *(array_t *) colors;

  last_max_clusters_ = 0;
  std::cout << "Done constructor!" << std::endl;
}

void visualization::SegmentationVisualizer::configureSegmentationViewer(PCLVisualizer &viewer)
{
  configureBaseViewer(viewer);

  viewer.setCameraPosition(0.414395, -0.134601, 0.669816, 0.190149, 0.0424081, 1.09322, -0.13325,
                            -0.93753, 0.321374);
  viewer.setCameraFieldOfView(0.8575);
  viewer.setCameraClipDistances(0.0067374, 6.7374);
  viewer.setPosition(637, 145);
  viewer.setSize(727, 619);
}

void visualization::SegmentationVisualizer::visualize()
{
  std::cout << "Init visualize!" << std::endl;
  PCLVisualizer viewer("Segmentation Viewer");

  int v1(0), v2(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  configureSegmentationViewer(viewer);

  std::cout << "Ready to spin!" << std::endl;

  while (!viewer.wasStopped())
  {
    std::cout << "Pre spin!" << std::endl;
    viewer.spinOnce(100);

    visualizeNormalsCloud(viewer, v1);
    visualizePlaneCloud(viewer, v1);
    visualizeOverTableCloud(viewer, v1);
    visualizeClusters(viewer, v2);
    std::cout << "Done spin!" << std::endl;
  }
}


void visualization::SegmentationVisualizer::visualizeNormalsCloud(PCLVisualizer &viewer, int viewport)
{
  boost::mutex::scoped_lock update_lock(segmentator_->update_normals_mutex_);
  // Check if cloud was updated (If not present program fails due to try to visualize zero size normals)
  if (segmentator_->point_clouds_updated_)
  {
    // Visualize normals
    viewer.removePointCloud("Normals");

    // Visualize plane (The add functions copy the data)
    PointCloudColorHandlerCustom<PointT> red_color(segmentator_->smoothed_cloud_, 255, 0, 0);
    if (!viewer.updatePointCloud(segmentator_->smoothed_cloud_, red_color, "Cloud"))
      viewer.addPointCloud<PointT>(segmentator_->smoothed_cloud_, red_color, "Cloud", viewport);

    viewer.addPointCloudNormals<PointT, Normal>(
        segmentator_->smoothed_cloud_, segmentator_->cloud_normals_,
        normals_count_, normals_size_, "Normals", viewport);

    segmentator_->point_clouds_updated_ = false;
  }
  update_lock.unlock();
}

void visualization::SegmentationVisualizer::visualizePlaneCloud(PCLVisualizer &viewer, int viewport)
{
  if (segmentator_->plane_updated_)
  {
    // Visualize plane
    PointCloudColorHandlerCustom<PointT> single_color(segmentator_->plane_cloud_, 0, 255, 0);
    if (!viewer.updatePointCloud(segmentator_->plane_cloud_, single_color, "plane"))
      viewer.addPointCloud<PointT>(segmentator_->plane_cloud_, single_color, "plane", viewport);

    PointT middle = segmentator_->plane_cloud_->points[(int) segmentator_->plane_cloud_->points.size() / 2];

    viewer.removeShape("real plane");
    viewer.addPlane(*(segmentator_->table_coefficients_), middle.x, middle.y, middle.z, "real plane", viewport);

    segmentator_->plane_updated_ = false;
  }
}

void visualization::SegmentationVisualizer::visualizeOverTableCloud(PCLVisualizer &viewer, int viewport)
{
  if (segmentator_->cloud_over_table_updated_)
  {
    // Visualize plane
    PointCloudColorHandlerCustom<PointT> blue_color(segmentator_->cloud_over_table_, 0, 0, 255);
    if (!viewer.updatePointCloud(segmentator_->cloud_over_table_, blue_color, "over table"))
      viewer.addPointCloud<PointT>(segmentator_->cloud_over_table_, blue_color, "over table", viewport);

    segmentator_->cloud_over_table_updated_ = false;
  }
}

void visualization::SegmentationVisualizer::visualizeClusters(PCLVisualizer &viewer, int viewport)
{
  // Check if cloud was updated
  if (segmentator_->clusters_updated_)
  {
    unsigned long size = segmentator_->cloud_cluster_vector_.size();

    // Check if there were more clusters than now and remove the older ones
    if (last_max_clusters_ > size)
    {
      for (unsigned long i = size; i < last_max_clusters_; i++)
        viewer.removePointCloud(generateName(i));
    }

    last_max_clusters_ = size;

    for (unsigned long i = 0; i < size; i++)
    {
      PointCloudTPtr cluster = segmentator_->cloud_cluster_vector_[i];
      std::string name = generateName(i);

      unsigned long j = i % 10;
      PointCloudColorHandlerCustom<PointT> rgb_color(
          cluster, colors_[j][0], colors_[j][1], colors_[j][2]);
      // Visualize cluster i
      if (!viewer.updatePointCloud(cluster, rgb_color, name))
        viewer.addPointCloud<PointT>(cluster, rgb_color, name, viewport);
    }
    segmentator_->clusters_updated_ = false;
  }
}

std::string visualization::SegmentationVisualizer::generateName(unsigned long i)
{
  std::stringstream ss;
  ss << "cluster " << i;
  std::string name = ss.str();

  return name;
}
} // namespace bachelors_final_project