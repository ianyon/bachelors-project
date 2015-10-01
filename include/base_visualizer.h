//
// Created by ian on 9/30/15.
//

#ifndef BACHELORS_FINAL_PROJECT_BASE_VISUALIZER_H
#define BACHELORS_FINAL_PROJECT_BASE_VISUALIZER_H

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

class BaseVisualizer
{

public:

  //! Function called for visualization thread
  virtual void visualize() = 0;

  void configureBaseViewer(pcl::visualization::PCLVisualizer&);

  //pcl::visualization::PCLVisualizer viewer;
};

}
}


#endif //BACHELORS_FINAL_PROJECT_BASE_VISUALIZER_H
