#include "segmentation_visualizer.h"

#include <pcl/common/centroid.h>

#include "cloud_segmentator.h"

using namespace pcl::visualization;

namespace bachelors_final_project
{

const std::string visualization::SegmentationVisualizer::ORIGIN_CLOUD = "Origin cloud";
const std::string visualization::SegmentationVisualizer::PLANE_SHAPE = "Plane shape";
const std::string visualization::SegmentationVisualizer::PLANE_NORMAL = "Normal line";
const std::string visualization::SegmentationVisualizer::ORIGIN_PLANE_NORMAL = "normal line base frame";
const std::string visualization::SegmentationVisualizer::ORIGIN_PLANE_NORMAL_RECONSTRUCTED = "normal line base frame reconstructed";
const std::string visualization::SegmentationVisualizer::PLANE_CLOUD = "Plane cloud";
const std::string visualization::SegmentationVisualizer::NORMALS_CLOUD = "Normals";
const std::string visualization::SegmentationVisualizer::CROPPED_CLOUD = "Cropped cloud";
const std::string visualization::SegmentationVisualizer::CLOUD_OVER_TABLE = "Over table";

visualization::SegmentationVisualizer::SegmentationVisualizer(segmentation::CloudSegmentator &segmentator) :
    BaseVisualizer("Segmentation visualizer"),
    segmentator_(segmentator),
    v1(0), v2(0)
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

  configure();
}


void visualization::SegmentationVisualizer::configure()
{
  BaseVisualizer::configure();

  createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  setCameraPosition(0.414395, -0.134601, 0.669816, 0.190149, 0.0424081, 1.09322, -0.13325, -0.93753, 0.321374);
  setCameraFieldOfView(0.8575);
  setCameraClipDistances(0.0067374, 6.7374);
  setPosition(637, 145);
  setSize(727, 619);

  ROS_INFO("Configured segmentation visualizer!");
}

void visualization::SegmentationVisualizer::computeSpinOnce()
{
  spinOnce(100);
  visualizeNormalsCloud(v1);
  visualizePlaneCloud(v1);
  visualizeOverTableCloud(v1);
  visualizeClusters(v2);
}

void visualization::SegmentationVisualizer::visualizeNormalsCloud(int viewport)
{
  boost::mutex::scoped_lock update_lock(obj().update_normals_mutex_);
  // Check if cloud was updated (If not present program fails due to try to visualize zero size normals)
  if (obj().point_clouds_updated_)
  {
    if (visualizeCloud(CROPPED_CLOUD, obj().cropped_cloud_, 255, 0, 0, viewport))
    {
      removePointCloud(NORMALS_CLOUD);
      // Convert to cm
      addPointCloudNormals<Point, Normal>(obj().cropped_cloud_, obj().cloud_normals_,
                                          normals_count_, normals_size_ / 100, NORMALS_CLOUD, viewport);
    }
    if (false)
    {
      visualizeCloud(ORIGIN_CLOUD, obj().cropped_cloud_base_frame, 255, 255, 0, viewport);
      Point empty;
      visualizeArrowFromCloudCentroid(ORIGIN_PLANE_NORMAL, obj().plane_normal_base_frame_, &empty,
                                      obj().cropped_cloud_base_frame, viewport);
      visualizeArrowFromCloudCentroid(ORIGIN_PLANE_NORMAL_RECONSTRUCTED, obj().normal_base_frame_reconstructed_, &empty,
                                      obj().cropped_cloud_base_frame, viewport);
    }

    obj().point_clouds_updated_ = false;
  }
  update_lock.unlock();
}

void visualization::SegmentationVisualizer::visualizePlaneCloud(int viewport)
{
  if (obj().plane_updated_)
  {
    removeShape(PLANE_SHAPE);
    removeShape(PLANE_NORMAL);

    if (visualizeCloud(PLANE_CLOUD, obj().plane_cloud_, 0, 255, 0, viewport))
    {
      Point middle;
      if (visualizeArrowFromCloudCentroid(PLANE_NORMAL, obj().plane_normal_kinect_frame_, &middle,
                                          obj().plane_cloud_, viewport))
        addPlane(*(obj().table_coefficients_), middle.x, middle.y, middle.z, PLANE_SHAPE, viewport);
    }
    obj().plane_updated_ = false;
  }
}

void visualization::SegmentationVisualizer::visualizeOverTableCloud(int viewport)
{
  if (obj().cloud_over_table_updated_)
  {
    visualizeCloud(CLOUD_OVER_TABLE, obj().cloud_over_table_, 0, 0, 255, viewport);
    obj().cloud_over_table_updated_ = false;
  }
}

void visualization::SegmentationVisualizer::visualizeClusters(int viewport)
{
  // Check if cloud was updated
  if (obj().clusters_updated_)
  {
    size_t size = obj().clusters_vector_.size();

    for (size_t i = 0; i < size; i++)
    {
      CloudPtr cluster = obj().getCluster(i);
      size_t j = i % 10;
      // Visualize cluster i
      visualizeCloud(generateName(i), cluster, colors_[j][0], colors_[j][1], colors_[j][2], viewport);
    }

    // Check if there were more clusters than now and remove the older ones
    for (size_t i = size; i < last_max_clusters_; i++)
      removePointCloud(generateName(i));

    last_max_clusters_ = size;
    obj().clusters_updated_ = false;
  }
}

std::string visualization::SegmentationVisualizer::generateName(size_t i)
{
  return boost::str(boost::format("cluster %lu") % i);
}

segmentation::CloudSegmentator &visualization::SegmentationVisualizer::obj()
{
  return segmentator_;
}

void visualization::SegmentationVisualizer::setParams(VisualizerParams &params)
{
  normals_count_ = params.normals_count;
  normals_size_ = params.normals_size;
}
} // namespace bachelors_final_project