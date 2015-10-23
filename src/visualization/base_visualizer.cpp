//
// Created by ian on 9/30/15.
//

#include "detection_visualizer.h"
#include "cloud_segmentator.h"
#include "segmentation_visualizer.h"
#include "base_visualizer.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>

using namespace pcl::visualization;

namespace bachelors_final_project
{

visualization::BaseVisualizer::BaseVisualizer(std::string name):
    PCLVisualizer(name)
{
}

void visualization::BaseVisualizer::configure()
{
  setBackgroundColor(0.1, 0.1, 0.1);
  // red (x), green (y), and blue (z).
  addCoordinateSystem(0.5);
}

bool visualization::BaseVisualizer::visualizeCloud(const std::string &id, CloudPtr &cloud, int r, int g,
                                                   int b, int viewport)
{
  if (cloud->size() == 0)
  {
    ROS_WARN("Tried to visualize a '%s' with no points", id.c_str());
    removePointCloud(id, viewport);
    return false;
  }

  PointCloudColorHandlerCustom<Point> color(cloud, r, g, b);

  // Visualize cloud (The add functions copy the data)
  if (!updatePointCloud(cloud, color, id))
    addPointCloud<Point>(cloud, color, id, viewport);

  return true;
}

void visualization::BaseVisualizer::visualizeArrow(std::string id, Point &point, Point &middle, int viewport)
{
  Point translated_point;
  translated_point.getVector4fMap() = point.getVector4fMap() + middle.getVector4fMap();

  removeShape(id);
  addArrow(middle, translated_point, 1.0, 0.94901961, 0.8, id, viewport);
}

bool visualization::BaseVisualizer::visualizeArrow(std::string id, Point &point, Point *middle, CloudPtr &cloud,
                                                   int viewport)
{
  if (cloud->size() == 0)
  {
    ROS_ERROR("Trying to visualize line '%s' with empty cloud coordinates", id.c_str());
    return false;
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  middle->getVector4fMap() = centroid;
  visualizeArrow(id, point, *middle, viewport);

  return true;
}

void visualization::BaseVisualizer::visualizePoint(pcl::PointXYZ point, int red, int green,
                                                   int blue,
                                                   std::string name)
{
  CloudPtr cloud(new Cloud);
  cloud->push_back(point);

  PointCloudColorHandlerCustom <Point> color(cloud, red, green, blue);
  if (!updatePointCloud(cloud, color, name))
    addPointCloud<Point>(cloud, color, name);
}

} // namespace bachelors_final_project