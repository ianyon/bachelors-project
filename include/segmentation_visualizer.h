#ifndef SEGMENTATION_VISUALIZER_H
#define SEGMENTATION_VISUALIZER_H


#include <string>
#include <ros/console.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "base_visualizer.h"


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
  SegmentationVisualizer(segmentation::CloudSegmentator &);

  void computeSpinOnce();

  void configure();

  void visualizeNormalsCloud(int viewport);

  void visualizePlaneCloud(int viewport);

  void visualizeOverTableCloud(int viewport);

  void visualizeClusters(int viewport);

  std::string generateName(unsigned long i);

  virtual void setParams(VisualizerParams &params);

  segmentation::CloudSegmentator & obj();

  segmentation::CloudSegmentator &segmentator_;

  int normals_count_;

  float normals_size_;

  int colors_[10][3];

  int v1, v2;

  unsigned long last_max_clusters_;

  static const std::string ORIGIN_CLOUD;
  static const std::string PLANE_SHAPE;
  static const std::string PLANE_NORMAL;
  static const std::string ORIGIN_PLANE_NORMAL;
  static const std::string ORIGIN_PLANE_NORMAL_RECONSTRUCTED;
  static const std::string PLANE_CLOUD;
  static const std::string NORMALS_CLOUD;
  static const std::string CROPPED_CLOUD;
  static const std::string CLOUD_OVER_TABLE;
  static const std::string VIZUALIZER_NAME;
};

// Needed to initialize bidimensional array without C++11
typedef struct
{
  int a[30];
} array_t;

} // namespace visualization
} // namespace bachelors_final_project

#endif // SEGMENTATION_VISUALIZER_H
