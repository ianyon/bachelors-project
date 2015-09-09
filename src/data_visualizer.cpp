#include "../include/data_visualizer.h"

DataVisualizer::DataVisualizer(DataHandler &data_handler):
  data_handler_(&data_handler)
{
}

void DataVisualizer::visualize()
{
  visualization::PCLVisualizer::Ptr viewer;
  viewer = visualization::PCLVisualizer::Ptr(new visualization::PCLVisualizer ("PCL Viewer"));
  viewer->setBackgroundColor (0.2, 0.2, 0.2);
  // red (x), green (y), and blue (z).
  viewer->addCoordinateSystem (0.5);

  viewer->setCameraPosition(0.414395, -0.134601, 0.669816, 0.190149, 0.0424081, 1.09322, -0.13325, -0.93753, 0.321374);
  viewer->setCameraFieldOfView(0.8575);
  viewer->setCameraClipDistances(0.0067374, 6.7374);
  viewer->setPosition(637, 145);
  viewer->setSize(727, 619);

  /*ModelCoefficients coeffs;
   coeffs.values.push_back (0.0);
   coeffs.values.push_back (0.0);
   coeffs.values.push_back (1.0);
   coeffs.values.push_back (0.0);
   viewer->addPlane (coeffs, "plane");*/

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);

    boost::mutex::scoped_lock updateLock(data_handler_->updateNormalsMutex);
    // Check if cloud was updated (If not present program fails due to try to visualize zero size normals)
    if(data_handler_->point_clouds_updated_)
    {
      // Visualize normals
      viewer->removePointCloud("Normals");

      // Visualize plane
      visualization::PointCloudColorHandlerCustom<PointXYZ> red_color(data_handler_->smoothed_cloud_, 255, 0, 0);
      if(!viewer->updatePointCloud(data_handler_->smoothed_cloud_, red_color, "Cloud"))
        viewer->addPointCloud<PointXYZ>(data_handler_->smoothed_cloud_, red_color, "Cloud");

      viewer->addPointCloudNormals<PointXYZ,Normal>(data_handler_->smoothed_cloud_,
                                                    data_handler_->cloud_normals_, viz_normals_count_,
                                                    normals_size_, "Normals");

      data_handler_->point_clouds_updated_ = false;
    }
    updateLock.unlock();

    if(data_handler_->plane_updated_)
    {
      // Visualize plane
      visualization::PointCloudColorHandlerCustom<PointXYZ> single_color(data_handler_->plane_cloud_, 0, 255, 0);
      if(!viewer->updatePointCloud(data_handler_->plane_cloud_, single_color, "plane"))
          viewer->addPointCloud<PointXYZ>(data_handler_->plane_cloud_,  single_color, "plane");

      data_handler_->plane_updated_ = false;
    }
  }
}
