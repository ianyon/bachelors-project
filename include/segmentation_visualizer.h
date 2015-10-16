#ifndef SEGMENTATION_VISUALIZER_H
#define SEGMENTATION_VISUALIZER_H


#include <string>
#include <ros/console.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "base_visualizer.h"
#include "definitions.h"


namespace bachelors_final_project
{

// Forward declarations
namespace segmentation
{
class CloudSegmentator;
}

namespace visualization
{
class SegmentationVisualizer : public BaseVisualizer
{
public:
  SegmentationVisualizer(segmentation::CloudSegmentator *);

  void visualize();

  void configureSegmentationViewer(pcl::visualization::PCLVisualizer&);

  void visualizeNormalsCloud(pcl::visualization::PCLVisualizer&, int);

  void visualizePlaneCloud(pcl::visualization::PCLVisualizer&, int);

  void visualizeOverTableCloud(pcl::visualization::PCLVisualizer&, int);

  void visualizeClusters(pcl::visualization::PCLVisualizer&, int);

  std::string generateName(unsigned long i);

  void visualizeCloud(const std::string &id, PointCloudTPtr &cloud, int r, int g, int b,
                      pcl::visualization::PCLVisualizer &viewer, int viewport);

  segmentation::CloudSegmentator *segmentator_;
  int normals_count_;
  float normals_size_;

  int colors_[10][3];

  unsigned long last_max_clusters_;
};

// Needed to initialize bidimensional array without C++11
typedef struct
{
  int a[30];
} array_t;

} // namespace visualization
} // namespace bachelors_final_project

#endif // SEGMENTATION_VISUALIZER_H
