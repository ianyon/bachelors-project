#include "data_handler.h"

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
    passThroughCloud.reset(new PointCloud<PointXYZ>());
    smoothed_cloud_.reset(new PointCloud<PointXYZ>());;
    cloud_normals_.reset(new PointCloud<PointXYZ>());;
}  // end DataHandler()


/*
 * Destructor
 */
DataHandler::~NodeExample()
{
} // end ~DataHandler()

void DataHandler::returnFromCallback(clock_t begin){
    //ros::Duration duration = ros::Time::now() - start;
    ROS_INFO("Callback took %gms", durationInMillis(begin));
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

    ROS_DEBUG_STREAM("PLANE FOUND. Model coefficients: " << coefficients->values[0] << " "
                                                                                    << coefficients->values[1] << " "
                                                                                    << coefficients->values[2] << " "
                                                                                    << coefficients->values[3]);

    ROS_DEBUG_STREAM("Model inliers: " << inliers->indices.size ());
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        ROS_DEBUG_STREAM(inliers->indices[i] << "    "
                         << input->points[inliers->indices[i]].x << " "
                                                                 << input->points[inliers->indices[i]].y << " "
                                                                 << input->points[inliers->indices[i]].z);
    }

    ROS_DEBUG("Plane segmentation took %gms", durationInMillis(begin));
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

void DataHandler::computeNormalsEfficiently(const PointCloud<PointXYZ>::Ptr &sensorCloud,
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

    ne.setMaxDepthChangeFactor(maxDepthChangeFactor);
    ne.setDepthDependentSmoothing(useDepthDependentSmoothing);
    ne.setNormalSmoothingSize(normalSmoothingSize);
    ne.setInputCloud(sensorCloud);
    ne.compute(*cloud_normals_);

    //removeNaNNormalsFromPointCloud(cloudin,cloudout,indexes);
    ROS_DEBUG("Integral image normals took %gms", durationInMillis(begin));
}

void DataHandler::projectOnPlane(const PointCloud<PointXYZ>::Ptr &sensorCloud, const ModelCoefficients::Ptr &tableCoefficients,
                                 const PointIndices::Ptr &tableInliers, PointCloud<PointXYZ>::Ptr &projectedTableCloud)
{
    clock_t begin = clock();
    ProjectInliers<PointXYZ> proj;
    proj.setModelType (SACMODEL_PLANE);
    proj.setIndices (tableInliers);
    proj.setInputCloud (sensorCloud);
    proj.setModelCoefficients (tableCoefficients);
    proj.filter (*projectedTableCloud);
    ROS_DEBUG("Project on plane took %gms", durationInMillis(begin));
}

void DataHandler::computeTableConvexHull(const PointCloud<PointXYZ>::Ptr& projectedTableCloud, PointCloud<PointXYZ>::Ptr& tableConvexHull)
{
    clock_t begin = clock();
    ConvexHull<PointXYZ> chull;
    chull.setInputCloud (projectedTableCloud);
    chull.reconstruct (*tableConvexHull);
    ROS_DEBUG("Convex hull took %gms", durationInMillis(begin));
    ROS_DEBUG("Convex hull has: %u data points.", tableConvexHull->points.size ());
}

bool DataHandler::extractCloudOverTheTable(const PointCloud<PointXYZ>::Ptr& sensorCloud,
                                           const PointCloud<PointXYZ>::Ptr& tableConvexHull,
                                           PointCloud<PointXYZ>::Ptr& cloudOverTheTable)
{
    clock_t begin = clock();
    // Segment those points that are in the polygonal prism
    ExtractPolygonalPrismData<PointXYZ> prism;
    // Objects must lie between minHeight and maxHeight m over the plane
    prism.setHeightLimits (minHeight, maxHeight);
    prism.setInputCloud (sensorCloud);
    prism.setInputPlanarHull (tableConvexHull);
    PointIndices::Ptr indicesOverTheTable (new PointIndices ());
    prism.segment (*indicesOverTheTable);

    if(indicesOverTheTable->indices.size () == 0)
    {
        ROS_DEBUG("Convex hull took %gms", durationInMillis(begin));
        ROS_WARN("No points over the table");
        return false;
    }

    // Extraxt indices over the table
    ExtractIndices<PointXYZ> extractIndices;
    extractIndices.setInputCloud(sensorCloud);
    extractIndices.setIndices(indicesOverTheTable);
    extractIndices.filter(*cloudOverTheTable);
    ROS_DEBUG("Convex hull took %gms", durationInMillis(begin));
    return true;
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

    ROS_DEBUG("Gaussian Smoothing took %gms", durationInMillis(begin));

    return smoothed_cloud_;
}

void DataHandler::cropPointCloud(const PointCloud<PointXYZ>::Ptr &cloudInput, PointCloud<PointXYZ>::Ptr &croppedCloud)
{
    // Kinect is 320/240
    Eigen::Vector4f minPoint;
    minPoint[0]=0;  // define minimum point x
    minPoint[1]=0;  // define minimum point y
    minPoint[2]=0;  // define minimum point z
    Eigen::Vector4f maxPoint;
    minPoint[0]=5;  // define max point x
    minPoint[1]=6;  // define max point y
    minPoint[2]=7;  // define max point z

    // Define translation and rotation ( this is optional)

    Eigen::Vector3f boxTranslatation;
    boxTranslatation[0]=1;
    boxTranslatation[1]=2;
    boxTranslatation[2]=3;
    // this moves your cube from (0,0,0)//minPoint to (1,2,3)  // maxPoint is now(6,8,10)

    CropBox<PointXYZ> cropFilter;
    cropFilter.setInputCloud (cloudInput);
    cropFilter.setMin(minPoint);
    cropFilter.setMax(maxPoint);
    cropFilter.setTranslation(boxTranslatation);

    cropFilter.filter (*croppedCloud);


    /*PassThrough<PointXYZ> pass;
  pass.setKeepOrganized(true);
  pass.setInputCloud (sensorCloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-yLimit, yLimit);
  pass.filter (*passThroughCloud);

  pass.setInputCloud (passThroughCloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-xLimit, xLimit);
  pass.filter (*passThroughCloud);

  pass.setInputCloud (passThroughCloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-zLimit, zLimit);
  pass.filter (*passThroughCloud);*/
}

void DataHandler::cropOrganizedPointCloud(const PointCloud<PointXYZ>::Ptr &cloudInput, PointCloud<PointXYZ>::Ptr &croppedCloud)
{
    clock_t begin = clock();
    // Kinect is 640/480
    int xRealSize = 64*scale,
            yRealSize = 48*scale;
    int xInit = fmax(cloudInput->width/2 - xRealSize/2 + xTranslate, 0),
            yInit = fmax(cloudInput->height/2 - yRealSize/2 + yTranslate, 0);
    int xEnd = fmin(xInit + xRealSize, cloudInput->width),
            yEnd = fmin(yInit + yRealSize, cloudInput->height);

    // Reserve space in the points vector to store the cloud
    croppedCloud->reserve(xRealSize*yRealSize);
    // Make que dimensions of the cloud be according the size of the vector
    croppedCloud->width = xRealSize;
    croppedCloud->height = yRealSize;
    croppedCloud->sensor_origin_ = cloudInput->sensor_origin_;
    croppedCloud->sensor_orientation_ = cloudInput->sensor_orientation_;

    for (size_t i=xInit; i < xEnd; ++i)
    {
        for (size_t j=yInit ; j < yEnd; ++j)
        {
            // We can't use croppedCloud->push_back because it breaks the organization
            croppedCloud->points.push_back(cloudInput->at(i, j));
        }
    }

    ROS_DEBUG("PointCloud cropping took %gms", durationInMillis(begin));
    ROS_DEBUG("Cropped from %u to %u", cloudInput->points.size(), croppedCloud->points.size());
}

void DataHandler::sensorCallback (const PCLPointCloud2::ConstPtr& sensorInput)
{
    ROS_INFO_ONCE("Callback Called");

    //ros::Time start = ros::Time::now();
    clock_t beginCallback = clock();

    PointCloud<PointXYZ>::Ptr sensorCloud(new PointCloud<PointXYZ>);
    fromPCLPointCloud2 (*sensorInput,*sensorCloud);

    if(sensorCloud->size()==0) return;

    passThroughCloud->clear();
    smoothed_cloud_->clear();
    cloud_normals_->clear();

    // PointCloud cropping
    cropOrganizedPointCloud(sensorCloud, passThroughCloud);

    // Gaussian smoothing
    gaussianSmoothing(passThroughCloud, smoothed_cloud_);
    publish(pubSmoothed, smoothed_cloud_);

    // Compute integral image normals
    computeNormalsEfficiently(smoothed_cloud_, cloud_normals_);
    update = true;

    // Segment plane using normals
    ModelCoefficients::Ptr tableCoefficients (new ModelCoefficients ());
    PointIndices::Ptr tableInliers (new PointIndices ());
    // Only continue if we find a plane
    if (!fitPlaneFromNormals(smoothed_cloud_, cloud_normals_, tableCoefficients, tableInliers))
        return returnFromCallback(beginCallback);

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
        return returnFromCallback(beginCallback);
    publish(pubObjects, cloudOverTheTable);

    // Clustering objects over the table
    //clusterObjects(cloudOverTheTable, 0.02, 100, 25000);
    //publish(pubObjects, );

    returnFromCallback(beginCallback);
}

