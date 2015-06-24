#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

//http://pointclouds.org/documentation/tutorials/hull_2d.php#hull-2d

using namespace pcl;
using namespace std;

int
main (int argc, char** argv)
{
  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>), 
                                      cloud_filtered (new PointCloud<PointXYZ>), 
                                      cloud_projected (new PointCloud<PointXYZ>);
  PCDReader reader;

  reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
  // Build a filter to remove spurious NaNs
  PassThrough<PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.1);
  pass.filter (*cloud_filtered);
  cerr << "PointCloud after filtering has: "
            << cloud_filtered->points.size () << " data points." << endl;

  ModelCoefficients::Ptr coefficients (new ModelCoefficients);
  PointIndices::Ptr inliers (new PointIndices);
  // Create the segmentation object
  SACSegmentation<PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (SACMODEL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);
  cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << endl;

  // Project the model inliers
  ProjectInliers<PointXYZ> proj;
  proj.setModelType (SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud_filtered);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << endl;

  // Create a Convex Hull representation of the projected inliers
  PointCloud<PointXYZ>::Ptr cloud_hull (new PointCloud<PointXYZ>);
  ConvexHull<PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  chull.reconstruct (*cloud_hull);

  cerr << "Convex hull has: " << cloud_hull->points.size ()
            << " data points." << endl;

  PCDWriter writer;
  writer.write ("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

  return (0);
}