#ifndef DATAVISUALIZER_H
#define DATAVISUALIZER_H

#include "data_handler.h"

class DataVisualizer
{
public:
  DataVisualizer(DataHandler &data_handler);

  //! Function called for visualization thread
  void visualize();

  DataHandler *data_handler_;
  int viz_normals_count_;
};

#endif // DATAVISUALIZER_H
