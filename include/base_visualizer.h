//
// Created by ian on 9/30/15.
//

#ifndef BACHELORS_FINAL_PROJECT_BASE_VISUALIZER_H
#define BACHELORS_FINAL_PROJECT_BASE_VISUALIZER_H

#include <string>

#include <ros/console.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "definitions.h"
#include "containers.h"

namespace pcl
{
namespace visualization
{
class PCLVisualizer;
}
}

namespace bachelors_final_project
{
namespace visualization
{

class BaseVisualizer : public pcl::visualization::PCLVisualizer
{
public:
  BaseVisualizer(const std::string name);

  virtual void computeSpinOnce() = 0;

  virtual void configure();

  virtual void setParams(VisualizerParams &params)
  { }

  bool visualizeCloud(const std::string &id, PointCloudPtr &cloud, int r, int g, int b, int viewport = 0);

  void visualizeArrow(std::string id, Point &point, Point &middle, int viewport = 0);

  bool visualizeArrow(std::string id, Point &point, Point *middle, PointCloudPtr &cloud, int viewport = 0);

  void visualizePoint(pcl::PointXYZ point, int red, int green, int blue, std::string name);
};

typedef boost::shared_ptr<BaseVisualizer> BaseVisualizerPtr;

}
}
#endif //BACHELORS_FINAL_PROJECT_BASE_VISUALIZER_H
