#ifndef DATAVISUALIZER_H
#define DATAVISUALIZER_H

#include <string>

#include <ros/console.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace bachelors_final_project
{
namespace segmentation
{

class DataHandler;

class DataVisualizer
{
public:
  DataVisualizer(DataHandler&);

  //! Function called for visualization thread
  void visualize();

  void visualizeNormalsCloud(pcl::visualization::PCLVisualizer::Ptr, int);
  void visualizePlaneCloud(pcl::visualization::PCLVisualizer::Ptr, int);
  void visualizeOverTableCloud(pcl::visualization::PCLVisualizer::Ptr, int);
  void visualizeClusters(pcl::visualization::PCLVisualizer::Ptr, int);
  std::string generateName(size_t i);

  DataHandler *data_handler_;
  int normals_count_;
  float normals_size_;
  int colors_[10][3];
  size_t last_max_clusters_;
};

// Needed to initialize bidimensional array without C++11
typedef struct{ int a[30]; } array_t;

} // namespace segmentation
} // namespace bachelors_final_project

#endif // DATAVISUALIZER_H
