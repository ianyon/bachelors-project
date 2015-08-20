#include "data_visualizer.h"

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
    viewer->addCoordinateSystem (0.3);
    viewer->initCameraParameters();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        // Check if cloud was updated (If not present program fails due to try to visualize zero size normals)
        if(data_handler_->point_clouds_updated)
        {
            // Visualize normals
            viewer->removePointCloud("Normals");

            if(!viewer->updatePointCloud(data_handler_->smoothedCloud, "Cloud"))
                viewer->addPointCloud<PointXYZ>(data_handler_->smoothedCloud, "Cloud");

            viewer->addPointCloudNormals<PointXYZ,Normal>(data_handler_->smoothed_cloud_,
                                                          data_handler_->cloud_normals_, viz_normals_count_,
                                                          1.0, "Normals");
            data_handler_->point_clouds_updated = false;
        }
    }
}

