#include <vector>

namespace bachelors_final_project
{
namespace detection
{

class GraspSampler
{
	GraspSampler();

	void sampleGraspingPoses(BoundingBox&, GraspTypesContainer*)

	void sampleSideGrasps(BoundingBox&, vector<Eigen::Vector3f>*);

	void sampleTopGrasps(BoundingBox&, vector<Eigen::Vector3f>*);

	vector<Eigen::Vector3f> side_grasps, top_grasps;
	float grasp_heigth;
	float ellipse_angular_step;
};

} // namespace detection
} // namespace bachelors_final_project
