#ifndef DATAVISUALIZER_H
#define DATAVISUALIZER_H

// Visualization
#include <pcl/visualization/pcl_visualizer.h>

#include "data_handler.h"

using namespace pcl;

class DataVisualizer
{
public:
  DataVisualizer(DataHandler &data_handler);

  //! Function called for visualization thread
  void visualize();

  DataHandler *data_handler_;
  int viz_normals_count_;
  float normals_size_;
};

#endif // DATAVISUALIZER_H
