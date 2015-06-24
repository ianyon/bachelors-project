#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

//http://www.pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php#normal-estimation-using-integral-images

using namespace pcl;

int
main ()
{
    // load point cloud
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

    // estimate normals
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);

    IntegralImageNormalEstimation<PointXYZ, Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    // visualize normals
    visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<PointXYZ,Normal>(cloud, normals);

    while (!viewer.wasStopped ())
    {
       viewer.spinOnce ();
    }
    return 0;
}