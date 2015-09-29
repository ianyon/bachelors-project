#include "data_visualizer.h"

#include "data_handler.h"

using namespace pcl;

namespace bachelors_final_project
{

segmentation::DataVisualizer::DataVisualizer(DataHandler &data_handler):
  data_handler_(&data_handler)
{
  const int colors[10][3] = { {170,57,57}, {170,96,57}, {37,112,89}, {45,136,45}, {128, 43, 102}, {170, 155, 57},
                        {69, 47, 116}, {132, 161, 54}, {137, 162, 54}, {170, 123, 57} };
 *(array_t*)colors_ = *(array_t*)colors;

  last_max_clusters_ = 0;
}

void segmentation::DataVisualizer::visualize()
{
  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer ("PCL Viewer"));

  int v1(0), v2(0);
  viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);

  viewer->setBackgroundColor (0.1, 0.1, 0.1);
  // red (x), green (y), and blue (z).
  viewer->addCoordinateSystem (0.5);

  viewer->setCameraPosition(0.414395, -0.134601, 0.669816, 0.190149, 0.0424081, 1.09322, -0.13325, -0.93753, 0.321374);
  viewer->setCameraFieldOfView(0.8575);
  viewer->setCameraClipDistances(0.0067374, 6.7374);
  viewer->setPosition(637, 145);
  viewer->setSize(727, 619);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);

    visualizeNormalsCloud(viewer, v1);

    visualizePlaneCloud(viewer, v1);

    visualizeOverTableCloud(viewer, v1);

    visualizeClusters(viewer, v2);
  }
}

void segmentation::DataVisualizer::visualizeNormalsCloud(visualization::PCLVisualizer::Ptr viewer, int viewport)
{
  clock_t begin = clock();
  boost::mutex::scoped_lock updateLock(data_handler_->update_normals_mutex_);
  // Check if cloud was updated (If not present program fails due to try to visualize zero size normals)
  if(data_handler_->point_clouds_updated_)
  {
    PointCloud<PointXYZ>::Ptr smoothed_cloud = data_handler_->smoothed_cloud_->makeShared();
    PointCloud<Normal>::Ptr cloud_normals = data_handler_->cloud_normals_->makeShared();
    data_handler_->point_clouds_updated_ = false;
    updateLock.unlock();

    // Visualize normals
    viewer->removePointCloud("Normals");

    // Visualize plane
    visualization::PointCloudColorHandlerCustom<PointXYZ> red_color(smoothed_cloud, 255, 0, 0);
    if (!viewer->updatePointCloud(smoothed_cloud, red_color, "Cloud"))
      viewer->addPointCloud<PointXYZ>(smoothed_cloud, red_color, "Cloud", viewport);

    viewer->addPointCloudNormals<PointXYZ, Normal>(
        smoothed_cloud, cloud_normals, normals_count_, normals_size_, "Normals", viewport);
  }
  else
  {
    updateLock.unlock();
  }
  ROS_INFO("VISUALIZATION took %gms\n\n", (double(clock() - begin) / CLOCKS_PER_SEC) * 1000);
}

void segmentation::DataVisualizer::visualizePlaneCloud(visualization::PCLVisualizer::Ptr viewer, int viewport)
{
  if(data_handler_->plane_updated_)
  {
    // Visualize plane
    visualization::PointCloudColorHandlerCustom<PointXYZ> single_color(data_handler_->plane_cloud_, 0, 255, 0);
    if(!viewer->updatePointCloud(data_handler_->plane_cloud_, single_color, "plane"))
      viewer->addPointCloud<PointXYZ>(data_handler_->plane_cloud_,  single_color, "plane", viewport);

    PointXYZ middle = data_handler_->plane_cloud_->points[(int)data_handler_->plane_cloud_->points.size()/2];

    viewer->removeShape("real plane");
    viewer->addPlane(*(data_handler_->table_coefficients_), middle.x, middle.y, middle.z, "real plane", viewport);

    data_handler_->plane_updated_ = false;
  }
}

void segmentation::DataVisualizer::visualizeOverTableCloud(visualization::PCLVisualizer::Ptr viewer, int viewport)
{
  if(data_handler_->cloud_over_table_updated_)
  {
    // Visualize plane
    visualization::PointCloudColorHandlerCustom<PointXYZ> blue_color(data_handler_->cloud_over_table_, 0, 0, 255);
    if(!viewer->updatePointCloud(data_handler_->cloud_over_table_, blue_color, "over table"))
        viewer->addPointCloud<PointXYZ>(data_handler_->cloud_over_table_,  blue_color, "over table", viewport);

    data_handler_->cloud_over_table_updated_ = false;
  }
}

void segmentation::DataVisualizer::visualizeClusters(visualization::PCLVisualizer::Ptr viewer, int viewport)
{
  // Check if cloud was updated
  if(data_handler_->clusters_updated_)
  {
    size_t size = data_handler_->cloud_cluster_vector_.size();

    // Check if there were more clusters than now and remove the older ones
    if(last_max_clusters_ > size)
    {
      for(size_t i = size; i < last_max_clusters_;i++)
        viewer->removePointCloud(generateName(i));
    }

    last_max_clusters_ = size;

    for(size_t i = 0; i < size;i++)
    {
      PointCloud<PointXYZ>::Ptr cluster = data_handler_->cloud_cluster_vector_[i];
      std::string name = generateName(i);

      size_t j = i%10;
      visualization::PointCloudColorHandlerCustom<PointXYZ> rgb_color(
            cluster, colors_[j][0], colors_[j][1], colors_[j][2]);
      // Visualize cluster i
      if(!viewer->updatePointCloud(cluster, rgb_color, name))
        viewer->addPointCloud<PointXYZ>(cluster, rgb_color, name, viewport);
    }
    data_handler_->clusters_updated_ = false;
  }
}

std::string segmentation::DataVisualizer::generateName(size_t i)
{
  std::stringstream ss;
  ss << "cluster " << i;
  std::string name = ss.str();

  return name;
}

} // namespace bachelors_final_project