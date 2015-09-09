#include "../include/data_handler.h"

/*
 * Constructor
 */
DataHandler::DataHandler(ros::NodeHandle nh)
{
    // Create a ROS publisher
    pubSmoothed = nh.advertise<PCLPointCloud2> ("downsampled", 1);
    pubPlanar = nh.advertise<PCLPointCloud2> ("planar", 1);
    pubObjects = nh.advertise<PCLPointCloud2> ("objects", 1);

    // Initialize pointers to point clouds
    pass_through_cloud_.reset(new PointCloud<PointXYZ>);
    smoothed_cloud_.reset(new PointCloud<PointXYZ>);
    plane_cloud_.reset(new PointCloud<PointXYZ>);
    cloud_normals_.reset(new PointCloud<Normal>);
}  // end DataHandler()


/*
 * Destructor
 */
DataHandler::~DataHandler()
{
} // end ~DataHandler()

void DataHandler::returnFromCallback(clock_t begin)
{
    //ros::Duration duration = ros::Time::now() - start;
    ROS_INFO("Callback took %gms\n\n", durationInMillis(begin));
}

void DataHandler::cropOrganizedPointCloud(const PointCloud<PointXYZ>::Ptr &cloudInput,
                                          PointCloud<PointXYZ>::Ptr &croppedCloud)
{
    clock_t begin = clock();
    // Kinect is 640/480
    int width = 64*scale,
        height = 48*scale;
    int init_col = fmax(cloudInput->width/2 - width/2 + xTranslate, 0),
        init_row = fmax(cloudInput->height/2 - height/2 + yTranslate, 0);
    //int end_col = fmin(xInit + width, cloudInput->width),
   //     end_row = fmin(yInit + height, cloudInput->height);

    // Reserve space in the points vector to store the cloud
    //croppedCloud->reserve(xRealSize*yRealSize);
    // Make que dimensions of the cloud be according the size of the vector
    croppedCloud->width = width;
    croppedCloud->height = height;
    // Change the size of the pointcloud
    croppedCloud->points.resize (width * height);
    croppedCloud->sensor_origin_ = cloudInput->sensor_origin_;
    croppedCloud->sensor_orientation_ = cloudInput->sensor_orientation_;

    for (unsigned int u = 0; u < width; u++)
    {
      for (unsigned int v = 0; v < height; v++)
      {
        // We can't use croppedCloud->push_back because it breaks the organization
        croppedCloud->at (u, v) = cloudInput->at (init_col + u, init_row + v);
      }
    }

    croppedCloud->is_dense = cloudInput->is_dense;

    ROS_INFO("PointCloud cropping took %gms", durationInMillis(begin));
    ROS_DEBUG("Cropped from %lu to %lu", cloudInput->points.size(), croppedCloud->points.size());
}

PointCloud<PointXYZ>::Ptr DataHandler::gaussianSmoothing(const PointCloud<PointXYZ>::Ptr &cloudInput, PointCloud<PointXYZ>::Ptr &smoothed_cloud_)
{
    clock_t begin = clock();
    //Set up the Gaussian Kernel
    filters::GaussianKernel<PointXYZ, PointXYZ>::Ptr kernel(new filters::GaussianKernel<PointXYZ, PointXYZ>());
    kernel->setSigma(gaussianSigma);
    kernel->setThresholdRelativeToSigma(3);

    //Set up the KDTree
    search::KdTree<PointXYZ>::Ptr kdTree(new search::KdTree<PointXYZ>);
    kdTree->setInputCloud(cloudInput);

    //Set up the Convolution Filter
    filters::Convolution3D<PointXYZ, PointXYZ, filters::GaussianKernel<PointXYZ, PointXYZ> > convolution;
    convolution.setKernel(*kernel);
    convolution.setInputCloud(cloudInput);
    convolution.setSearchMethod(kdTree);
    convolution.setRadiusSearch(gaussianSearchRadius);
    convolution.convolve(*smoothed_cloud_);

    ROS_INFO("Gaussian Smoothing took %gms", durationInMillis(begin));

    return smoothed_cloud_;
}


void DataHandler::computeNormalsEfficiently(const PointCloud<PointXYZ>::Ptr &sensor_cloud,
                                            PointCloud<Normal>::Ptr &cloud_normals_)
{
    clock_t begin = clock();
    IntegralImageNormalEstimation<PointXYZ, Normal> ne;

    switch (normalEstimationMethod)
    {
        case 1:
            ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
            break;
        case 2:
            ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
            break;
        case 3:
            ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
            break;
        case 4:
            ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
            break;
        default:
            ROS_ERROR("Wrong Normal estimation method. Parameter error");
            ros::shutdown();
            return;
    }

    /*
 // fill the cloud somehow...
    points3dToPointsPcl(points, nPoints, cloud);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setNumberOfThreads(8);
    ne.setRadiusSearch (0.15);
    ne.compute (*cloud_normals);
*/

    ne.setMaxDepthChangeFactor(maxDepthChangeFactor);
    ne.setDepthDependentSmoothing(useDepthDependentSmoothing);
    ne.setNormalSmoothingSize(normalSmoothingSize);
    ne.setInputCloud(sensor_cloud);
    //ne.setKSearch(0);
    ne.compute(*cloud_normals_);

    //removeNaNNormalsFromPointCloud(cloudin,cloudout,indexes);
    ROS_INFO("Integral image normals took %gms", durationInMillis(begin));
}

/* Use SACSegmentation to find the dominant plane in the scene
 * Inputs:
 *   input
 *     The input point cloud
 *   distanceThreshold
 *     The inlier/outlier threshold.  Points within this distance
 *     from the hypothesized plane are scored as inliers.
 * Return: A pointer to the ModelCoefficients (i.e., the 4 coefficients of the plane,
 *         represented in c0*x + c1*y + c2*z + c3 = 0 form)
 */
bool DataHandler::fitPlaneFromNormals (const PointCloud<PointXYZ>::Ptr &input, PointCloud<Normal>::Ptr &normals,
                                       ModelCoefficients::Ptr &coefficients, PointIndices::Ptr &inliers)
{
    clock_t begin = clock();
    // Intialize the SACSegmentationFromNormals object
    SACSegmentationFromNormals<PointXYZ, Normal> seg;
    seg.setModelType (SACMODEL_NORMAL_PLANE);
    seg.setMethodType (SAC_RANSAC);

    /************** SACSegmentationFromNormals **************/
    // Set the relative weight (between 0 and 1) to give to the angular distance
    // (0 to pi/2) between point normals and the plane normal.
    seg.setNormalDistanceWeight (normalDistanceWeight);

    // Set the minimum opning angle for a cone model.
    //seg.setMinMaxOpeningAngle (minAngle, maxAngle);

    // Set the distance we expect a plane model to be from the origin. I hadn't found implementation details.
    //seg.setDistanceFromOrigin(originDistance);

    /******************* SACSegmentation ********************/
    seg.setDistanceThreshold (distanceThreshold);
    seg.setMaxIterations (maxIterations);
    seg.setOptimizeCoefficients (optimizeCoefficients);

    // Set the probability of choosing at least one sample free from outliers.
    //seg.setProbability (probability);

    // Set the maximum distance allowed when drawing random samples.
    //SACSegmentation<PointXYZ>::SearchPtr search(new search::KdTree<PointXYZ>);
    //seg.setSamplesMaxDist (sampleMaxDistance, search);

    if(useSpecificPlane){
        Eigen::Vector3f axis = Eigen::Vector3f(planeX,planeY,planeZ);
        // Set the axis along which we need to search for a model perpendicular to.
        seg.setAxis(axis);
    }

    // Set the angle epsilon (delta) threshold. The maximum allowed difference between the model
    // normal and the given axis in radians. Specify the angle of the normals of the above plane
    seg.setEpsAngle(epsAngle);

    seg.setInputCloud (input);
    seg.setInputNormals (normals);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0) return false;

    ROS_DEBUG_STREAM("PLANE FOUND. Model coefficients: " << coefficients->values[0]
        << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3]);

    ROS_DEBUG_STREAM("Model inliers: " << inliers->indices.size ());

    ROS_INFO("Plane segmentation took %gms", durationInMillis(begin));
    return true;
}

void DataHandler::extractPlaneCloud (const PointCloud<PointXYZ>::Ptr &input, PointIndices::Ptr &inliers)
{
  // Create the filtering object
  ExtractIndices<PointXYZ> extract;

  // Extract the inliers
  extract.setInputCloud (input);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*plane_cloud_);
  ROS_DEBUG("Extracted PointCloud representing the planar component");
}

void DataHandler::projectOnPlane(const PointCloud<PointXYZ>::Ptr &sensor_cloud,
                                 const ModelCoefficients::Ptr &tableCoefficients,
                                 const PointIndices::Ptr &tableInliers,
                                 PointCloud<PointXYZ>::Ptr &projectedTableCloud)
{
    clock_t begin = clock();
    ProjectInliers<PointXYZ> proj;
    proj.setModelType (SACMODEL_PLANE);
    proj.setIndices (tableInliers);
    proj.setInputCloud (sensor_cloud);
    proj.setModelCoefficients (tableCoefficients);
    proj.filter (*projectedTableCloud);
    ROS_INFO("Project on plane took %gms", durationInMillis(begin));
}

void DataHandler::computeTableConvexHull(const PointCloud<PointXYZ>::Ptr& projectedTableCloud,
                                         PointCloud<PointXYZ>::Ptr& tableConvexHull)
{
    clock_t begin = clock();
    ConvexHull<PointXYZ> chull;
    chull.setInputCloud (projectedTableCloud);
    chull.reconstruct (*tableConvexHull);    
    tableConvexHull->push_back(tableConvexHull->at(0));
    ROS_INFO("Convex hull took %gms", durationInMillis(begin));
    ROS_DEBUG("Convex hull has: %lu data points.", tableConvexHull->points.size ());
}

bool DataHandler::extractCloudOverTheTable(const PointCloud<PointXYZ>::Ptr& sensor_cloud,
                                           const PointCloud<PointXYZ>::Ptr& tableConvexHull,
                                           PointCloud<PointXYZ>::Ptr& cloudOverTheTable)
{
    clock_t begin = clock();
    // Segment those points that are in the polygonal prism
    ExtractPolygonalPrismData<PointXYZ> prism;
    // Objects must lie between minHeight and maxHeight m over the plane
    prism.setHeightLimits (minHeight, maxHeight);
    prism.setInputCloud (sensor_cloud);
    prism.setInputPlanarHull (tableConvexHull);
    PointIndices::Ptr indicesOverTheTable (new PointIndices ());
    prism.segment (*indicesOverTheTable);

    if(indicesOverTheTable->indices.size () == 0)
    {
        ROS_INFO("Extract cloud over the table took %gms", durationInMillis(begin));
        ROS_WARN("No points over the table");
        return false;
    }

    // Extraxt indices over the table
    ExtractIndices<PointXYZ> extractIndices;
    extractIndices.setInputCloud(sensor_cloud);
    extractIndices.setIndices(indicesOverTheTable);
    extractIndices.filter(*cloudOverTheTable);
    ROS_INFO("Extract cloud over the table took %gms", durationInMillis(begin));
    return true;
}


/* Use EuclidieanClusterExtraction to group a cloud into contiguous clusters
 * Inputs:
 *   input
 *     The input point cloud
 *   cluster_tolerance
 *     The maximum distance between neighboring points in a cluster [cm]
 *   min/max_cluster_size
 *     The minimum and maximum allowable cluster sizes
 * Return (by reference): a vector of PointIndices containing the points indices in each cluster
 */
bool DataHandler::clusterObjects (const PointCloud<PointXYZ>::Ptr & cloudOverTheTable, float clusterTolerance,
                                  int minClusterSize, int maxClusterSize)
{
    // Creating the KdTree object for the search method of the extraction
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>);
    tree->setInputCloud (cloudOverTheTable);

    EuclideanClusterExtraction<PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);

    ec.setSearchMethod (tree);
    ec.setInputCloud (cloudOverTheTable);

    vector<PointIndices> clusterIndices;

    ec.extract (clusterIndices);

    vector<PointCloud<PointXYZ>::Ptr > cloudClusterVector;

    int j = 0;
    for (vector<PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {
        PointCloud<PointXYZ>::Ptr cloudCluster (new PointCloud<PointXYZ>);

        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloudCluster->points.push_back (cloudOverTheTable->points[*pit]);

        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        // Store the clusters in a vector. It's the best way?
        cloudClusterVector.push_back (cloudCluster);

        ROS_DEBUG_STREAM("PointCloud representing the Cluster: " << cloudCluster->points.size () << " data points.");
        stringstream ss;
        ss << "cloudCluster_" << j << ".pcd";
        //writer.write<PointXYZ> (ss.str (), *cloudCluster, false);
        j++;
    }
}


void DataHandler::sensorCallback (const PCLPointCloud2::ConstPtr& sensorInput)
{
    ROS_INFO_ONCE("Callback Called");

    //ros::Time start = ros::Time::now();
    clock_t beginCallback = clock();

    PointCloud<PointXYZ>::Ptr sensor_cloud(new PointCloud<PointXYZ>);
    fromPCLPointCloud2 (*sensorInput,*sensor_cloud);

    if(sensor_cloud->size()==0) return;

    boost::mutex::scoped_lock updateLock(updateNormalsMutex); // Init smoothed_cloud_ and cloud_normals_ mutex

    pass_through_cloud_->clear();
    smoothed_cloud_->clear();
    cloud_normals_->clear();

    point_clouds_updated_ = true;

    // PointCloud cropping
    cropOrganizedPointCloud(sensor_cloud, smoothed_cloud_);

    // Gaussian smoothing
    //gaussianSmoothing(pass_through_cloud_, smoothed_cloud_);
    //publish(pubSmoothed, smoothed_cloud_);

    // Compute integral image normals
    PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
    computeNormalsEfficiently(smoothed_cloud_, cloud_normals_);

    updateLock.unlock(); // End smoothed_cloud_ and cloud_normals_ mutex

    // Segment plane using normals
    ModelCoefficients::Ptr tableCoefficients (new ModelCoefficients ());
    PointIndices::Ptr tableInliers (new PointIndices ());
    // Only continue if we find a plane
    if (!fitPlaneFromNormals(smoothed_cloud_, cloud_normals_, tableCoefficients, tableInliers))
        return returnFromCallback(beginCallback);

    extractPlaneCloud(smoothed_cloud_, tableInliers);
    plane_updated_ = true;

    // Project plane points (inliers) in model plane
    PointCloud<PointXYZ>::Ptr projectedTableCloud (new PointCloud<PointXYZ>);
    projectOnPlane(smoothed_cloud_, tableCoefficients, tableInliers, projectedTableCloud);
    publish(pubPlanar, projectedTableCloud);

    // Create a Convex Hull representation of the projected inliers
    PointCloud<PointXYZ>::Ptr tableConvexHull(new PointCloud<PointXYZ>);
    computeTableConvexHull(projectedTableCloud, tableConvexHull);

    // Extract points over the table's convex hull
    PointCloud<PointXYZ>::Ptr cloudOverTheTable(new PointCloud<PointXYZ>);
    // Only continue if there are points over the table
    if(!extractCloudOverTheTable(smoothed_cloud_, tableConvexHull, cloudOverTheTable))
    {        
        return returnFromCallback(beginCallback);
    }
    publish(pubObjects, cloudOverTheTable);

    // Clustering objects over the table
    //clusterObjects(cloudOverTheTable, 0.02, 100, 25000);
    //publish(pubObjects, );

    returnFromCallback(beginCallback);
}

