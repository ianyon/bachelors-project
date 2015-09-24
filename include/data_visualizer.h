#ifndef DATAVISUALIZER_H
#define DATAVISUALIZER_H

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

#include <ros/console.h>

#include "data_handler.h"

using namespace pcl;

class DataVisualizer
{
public:
  DataVisualizer(DataHandler &data_handler);

  //! Function called for visualization thread
  void visualize();

  void visualizeNormalsCloud(visualization::PCLVisualizer::Ptr viewer, int viewport);
  void visualizePlaneCloud(visualization::PCLVisualizer::Ptr viewer, int viewport);
  void visualizeOverTableCloud(visualization::PCLVisualizer::Ptr viewer, int viewport);
  void visualizeClusters(visualization::PCLVisualizer::Ptr viewer, int viewport);
  string generateName(size_t i);

  DataHandler *data_handler_;
  int normals_count_;
  float normals_size_;
  int colors_[10][3];
  int last_max_clusters_;
};

// Needed to initialize bidimensional array without C++11
typedef struct{ int a[30]; } array_t;

#endif // DATAVISUALIZER_H
