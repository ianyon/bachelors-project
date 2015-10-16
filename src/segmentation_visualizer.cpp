#include "segmentation_visualizer.h"

#include <pcl/common/centroid.h>

#include "cloud_segmentator.h"

using namespace pcl::visualization;

namespace bachelors_final_project
{

static const std::string ORIGIN_CLOUD_ID = "origin cloud";
static const std::string PLANE_SHAPE_ID = "real plane";
static const std::string PLANE_NORMAL_ID = "normal line";
static const std::string PLANE_NORMAL_BASE_FRAME_ID = "normal line base frame";
static const std::string PLANE_CLOUD_ID = "plane";
static const std::string NORMALS_CLOUD_ID = "Normals";
static const std::string CROPPED_CLOUD_ID = "Cloud";
static const std::string CLOUD_OVER_TABLE_ID = "over table";

visualization::SegmentationVisualizer::SegmentationVisualizer(segmentation::CloudSegmentator *segmentator) :
    segmentator_(segmentator)
{
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
  ROS_INFO("Intitiated segmentation visualizer thread!");
  PCLVisualizer viewer("Segmentation Viewer");

  int v1(0), v2(0);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  configureSegmentationViewer(viewer);

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);

    visualizeNormalsCloud(viewer, v1);
    visualizePlaneCloud(viewer, v1);
    visualizeOverTableCloud(viewer, v1);
    visualizeClusters(viewer, v2);
  }
}

void visualization::SegmentationVisualizer::visualizeNormalsCloud(PCLVisualizer &viewer, int viewport)
{
  boost::mutex::scoped_lock update_lock(segmentator_->update_normals_mutex_);
  // Check if cloud was updated (If not present program fails due to try to visualize zero size normals)
  if (segmentator_->point_clouds_updated_)
  {
    visualizeCloud(CROPPED_CLOUD_ID, segmentator_->cropped_cloud_, 255, 0, 0, viewer, viewport);

    // Visualize normals
    viewer.removePointCloud(NORMALS_CLOUD_ID);
    viewer.addPointCloudNormals<PointT, Normal>(segmentator_->cropped_cloud_, segmentator_->cloud_normals_,
        normals_count_, normals_size_, NORMALS_CLOUD_ID, viewport);

    if(false)
    {
      visualizeCloud(ORIGIN_CLOUD_ID, segmentator_->cropped_cloud_base_frame, 255, 255, 0, viewer, viewport);
      visualizeLine(PLANE_NORMAL_BASE_FRAME_ID, segmentator_->plane_normal_base_frame_,
                    segmentator_->cropped_cloud_base_frame, viewer, viewport);
    }

    segmentator_->point_clouds_updated_ = false;
  }
  update_lock.unlock();
}

PointT visualization::SegmentationVisualizer::visualizeLine(std::string id, PointT &point, PointCloudTPtr &cloud,
                                                            PCLVisualizer &viewer, int viewport)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  PointT middle;
  middle.getVector4fMap() = centroid;

  PointT translated_point;
  translated_point.getVector4fMap() = point.getVector4fMap() + middle.getVector4fMap();

  viewer.removeShape(id);
  viewer.addLine(middle, translated_point, id, viewport);
  return middle;
}

void visualization::SegmentationVisualizer::visualizePlaneCloud(PCLVisualizer &viewer, int viewport)
{
  if (segmentator_->plane_updated_)
  {
    viewer.removeShape(PLANE_SHAPE_ID);
    if (segmentator_->plane_cloud_->size() == 0)
    {
      viewer.removePointCloud(PLANE_CLOUD_ID, viewport);
      segmentator_->plane_updated_ = false;
      return;
    }

    // Visualize plane
    visualizeCloud(PLANE_CLOUD_ID, segmentator_->plane_cloud_, 0, 255, 0, viewer, viewport);

    PointT middle = visualizeLine(PLANE_NORMAL_ID, segmentator_->plane_normal_kinect_frame_,
                                  segmentator_->plane_cloud_, viewer, viewport);

    viewer.addPlane(*(segmentator_->table_coefficients_), middle.x, middle.y, middle.z, PLANE_SHAPE_ID, viewport);

    segmentator_->plane_updated_ = false;
  }
}

void visualization::SegmentationVisualizer::visualizeOverTableCloud(PCLVisualizer &viewer, int viewport)
{
  if (segmentator_->cloud_over_table_updated_)
  {
    // Visualize cloud_over_table
    visualizeCloud(CLOUD_OVER_TABLE_ID, segmentator_->cloud_over_table_, 0, 0, 255, viewer, viewport);

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
      unsigned long j = i % 10;
      // Visualize cluster i
      visualizeCloud(generateName(i), cluster, colors_[j][0], colors_[j][1], colors_[j][2], viewer, viewport);
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

void visualization::SegmentationVisualizer::visualizeCloud(
    const std::string &id, PointCloudTPtr &cloud, int r, int g, int b, PCLVisualizer &viewer, int viewport)
{
  if(cloud->size()==0)
  {
    ROS_WARN("Tried to visualize a cloud %s with no points", id.c_str());
    return;
  }

  PointCloudColorHandlerCustom<PointT> color(cloud, r, g, b);

  // Visualize cloud (The add functions copy the data)
  if (!viewer.updatePointCloud(cloud, color, id))
    viewer.addPointCloud<PointT>(cloud, color, id, viewport);
}

} // namespace bachelors_final_project