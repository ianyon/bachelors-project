#include "segmentation_visualizer.h"

#include <pcl/common/centroid.h>

#include "cloud_segmentator.h"

using namespace pcl::visualization;

namespace bachelors_final_project
{

static const std::string ORIGIN_CLOUD = "Origin cloud";
static const std::string PLANE_SHAPE = "Plane shape";
static const std::string PLANE_NORMAL = "Normal line";
static const std::string ORIGIN_PLANE_NORMAL = "normal line base frame";
static const std::string ORIGIN_PLANE_NORMAL_RECONSTRUCTED = "normal line base frame reconstructed";
static const std::string PLANE_CLOUD = "Plane cloud";
static const std::string NORMALS_CLOUD = "Normals";
static const std::string CROPPED_CLOUD = "Cropped cloud";
static const std::string CLOUD_OVER_TABLE = "Over table";

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
  boost::mutex::scoped_lock update_lock(obj().update_normals_mutex_);
  // Check if cloud was updated (If not present program fails due to try to visualize zero size normals)
  if (obj().point_clouds_updated_)
  {
    if(visualizeCloud(CROPPED_CLOUD, obj().cropped_cloud_, 255, 0, 0, viewer, viewport))
    {
      viewer.removePointCloud(NORMALS_CLOUD);
      // Convert to cm
      viewer.addPointCloudNormals<Point, Normal>(obj().cropped_cloud_, obj().cloud_normals_,
                                                 normals_count_, normals_size_ / 100, NORMALS_CLOUD, viewport);
    }
    if (false)
    {
      visualizeCloud(ORIGIN_CLOUD, obj().cropped_cloud_base_frame, 255, 255, 0, viewer, viewport);
      Point empty;
      visualizeLine(ORIGIN_PLANE_NORMAL, obj().plane_normal_base_frame_,
                    obj().cropped_cloud_base_frame, viewer, viewport, &empty);
      visualizeLine(ORIGIN_PLANE_NORMAL_RECONSTRUCTED, obj().normal_base_frame_reconstructed_,
                    obj().cropped_cloud_base_frame, viewer, viewport, &empty);
    }

    obj().point_clouds_updated_ = false;
  }
  update_lock.unlock();
}

void visualization::SegmentationVisualizer::visualizePlaneCloud(PCLVisualizer &viewer, int viewport)
{
  if (obj().plane_updated_)
  {
    viewer.removeShape(PLANE_SHAPE);
    viewer.removeShape(PLANE_NORMAL);

    if (visualizeCloud(PLANE_CLOUD, obj().plane_cloud_, 0, 255, 0, viewer, viewport))
    {
      Point middle;
      if (visualizeLine(PLANE_NORMAL, obj().plane_normal_kinect_frame_, obj().plane_cloud_,
                        viewer, viewport, &middle))
        viewer.addPlane(*(obj().table_coefficients_), middle.x, middle.y, middle.z, PLANE_SHAPE, viewport);
    }
    obj().plane_updated_ = false;
  }
}

void visualization::SegmentationVisualizer::visualizeOverTableCloud(PCLVisualizer &viewer, int viewport)
{
  if (obj().cloud_over_table_updated_)
  {
    visualizeCloud(CLOUD_OVER_TABLE, obj().cloud_over_table_, 0, 0, 255, viewer, viewport);
    obj().cloud_over_table_updated_ = false;
  }
}

void visualization::SegmentationVisualizer::visualizeClusters(PCLVisualizer &viewer, int viewport)
{
  // Check if cloud was updated
  if (obj().clusters_updated_)
  {
    unsigned long size = obj().clusters_vector_.size();

    // Check if there were more clusters than now and remove the older ones
    if (last_max_clusters_ > size)
    {
      for (unsigned long i = size; i < last_max_clusters_; i++)
        viewer.removePointCloud(generateName(i));
    }

    last_max_clusters_ = size;

    for (unsigned long i = 0; i < size; i++)
    {
      PointCloudPtr cluster = obj().clusters_vector_[i];
      unsigned long j = i % 10;
      // Visualize cluster i
      visualizeCloud(generateName(i), cluster, colors_[j][0], colors_[j][1], colors_[j][2], viewer, viewport);
    }
    obj().clusters_updated_ = false;
  }
}

std::string visualization::SegmentationVisualizer::generateName(unsigned long i)
{
  return boost::str(boost::format("cluster %lu") % i);
}

bool visualization::SegmentationVisualizer::visualizeCloud(
    const std::string &id, PointCloudPtr &cloud, int r, int g, int b, PCLVisualizer &viewer, int viewport)
{
  if (cloud->size() == 0)
  {
    ROS_WARN("Tried to visualize a '%s' with no points", id.c_str());
    viewer.removePointCloud(id, viewport);
    return false;
  }

  PointCloudColorHandlerCustom<Point> color(cloud, r, g, b);

  // Visualize cloud (The add functions copy the data)
  if (!viewer.updatePointCloud(cloud, color, id))
    viewer.addPointCloud<Point>(cloud, color, id, viewport);

  return true;
}

bool visualization::SegmentationVisualizer::visualizeLine(std::string id, Point &point, PointCloudPtr &cloud,
                                                          PCLVisualizer &viewer, int viewport, Point *middle)
{
  if (cloud->size() == 0)
  {
    ROS_ERROR("Trying to visualize line '%s' with empty cloud coordinates", id.c_str());
    return false;
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  middle->getVector4fMap() = centroid;

  Point translated_point;
  translated_point.getVector4fMap() = point.getVector4fMap() + middle->getVector4fMap();

  viewer.removeShape(id);
  viewer.addArrow(*middle, translated_point, 1.0, 0.94901961, 0.8, id, viewport);
  return true;
}

segmentation::CloudSegmentator & visualization::SegmentationVisualizer::obj()
{
  return *segmentator_;
}
} // namespace bachelors_final_project