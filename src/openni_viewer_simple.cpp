 #include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
 #include <pcl/io/openni_grabber.h>
 #include <pcl/visualization/cloud_viewer.h>

class SimpleOpenNIViewer
{
  public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
      // Initialized once (2 cycles overhead each!!)
      static unsigned count = 0;
      static double last = pcl::getTime ();

      // Processing

      if (!viewer.wasStopped())
        viewer.showCloud (cloud);

      // End Processing

      if (++count == 30)
      {
        double now = pcl::getTime ();
        double framerate = double(count)/double(now-last);
        int middlePoint = (cloud->width >> 1)*(cloud->height + 1);

        std::cout << "distance of center pixel :" << cloud->points[middlePoint].z
          << " mm. Average framerate: " << framerate << " Hz" <<  std::endl;
        count = 0;
        last = now;
      }      
    }

    void run ()
    {
      // create a new grabber for OpenNI devices
      pcl::Grabber* interface = new pcl::OpenNIGrabber();

      // make callback function from member function
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

      // connect callback function for desired signal. In this case its a point cloud with color values
      boost::signals2::connection c = interface->registerCallback (f);

      // start receiving point clouds
      interface->start ();

      // Disconnect stream
      //if (c.connected ())
      //  c.disconnect ();

      // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
      while (!viewer.wasStopped())
        boost::this_thread::sleep (boost::posix_time::seconds (1));

      // stop the grabber
      interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};

int main ()
{
  SimpleOpenNIViewer v;
  v.run ();
  return 0;
}