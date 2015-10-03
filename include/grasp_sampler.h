#include <vector>
#include <Eigen/Core>
#include "grasp_point_detector.h"

namespace bachelors_final_project
{
namespace detection
{

class GraspSampler
{
	GraspSampler();

	void sampleGraspingPoses(GraspPointDetector::BoundingBox&, GraspPointDetector::GraspTypesContainer*);

	void sampleSideGrasps(GraspPointDetector::BoundingBox &, std::vector<PointT> *);

	void sampleTopGrasps(GraspPointDetector::BoundingBox&, std::vector<PointT>*);

	std::vector<PointT> side_grasps, top_grasps;
	float grasp_heigth;
	float ellipse_angular_step;
  std::vector<float> theta_array;
};

} // namespace detection
} // namespace bachelors_final_project
