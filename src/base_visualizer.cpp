//
// Created by ian on 9/30/15.
//

#include "base_visualizer.h"

#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::visualization;
namespace bachelors_final_project
{
void visualization::BaseVisualizer::configureBaseViewer(PCLVisualizer &viewer) {
  viewer.setBackgroundColor(0.1, 0.1, 0.1);
  // red (x), green (y), and blue (z).
  viewer.addCoordinateSystem(0.5);
}
} // namespace bachelors_final_project