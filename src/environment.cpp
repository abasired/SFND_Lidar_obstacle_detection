/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  
  // Step 1: filtering cloud using crop box and voxel filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25 , Eigen::Vector4f (30, 8, 0, 1), Eigen::Vector4f ( -10, -6, -2, 1));
    //renderPointCloud(viewer,filterCloud,"filterCloud");

  // step 2: Segment filtered point cloud into road and obstacles
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    //renderPointCloud( viewer,  SegmentCloud.first, "Segmented obstacle cloud", Color(1,0,0));
    renderPointCloud( viewer,  SegmentCloud.second, "Segmented plane cloud", Color(0,1,0));

  // step 3: clustering points corresponding to various obstacles 
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(SegmentCloud.first, 0.3, 10, 500);

    //Box box = pointProcessorI->BoundingBox(filterCloud);
    //renderBox(viewer, box, 0);

    // step 4: Bounding box corresponding to various clusters 
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){

        std::cout<<"cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
    
  
  
}
void cityBlock_old(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");
  
  // Step 1: filtering cloud using crop box and voxel filter
  
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25 , Eigen::Vector4f (30, 8, 0, 1), Eigen::Vector4f ( -10, -6, -2, 1));
    //renderPointCloud(viewer,filterCloud,"filterCloud");

  // step 2: Segment filtered point cloud into road and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentCloud = pointProcessorI->SegmentPlane_ransac(filterCloud, 100, 0.2);
    //renderPointCloud( viewer,  SegmentCloud.first, "Segmented obstacle cloud", Color(1,0,0));
    //renderPointCloud( viewer,  SegmentCloud.second, "Segmented plane cloud", Color(0,1,0));

  // step 3: clustering points corresponding to various obstacles 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(SegmentCloud.first, 0.3, 10, 500);

    //Box box = pointProcessorI->BoundingBox(filterCloud);
    //renderBox(viewer, box, 0);

 // step 4: Bounding box corresponding to various clusters 
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){

        std::cout<<"cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
    
  
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();
    //renderRays( viewer, lidar->position,  cloud);
    //renderPointCloud(viewer, cloud, "PCD", Color(1,1,1));

    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pt_proc; 
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentCloud = pt_proc.SegmentPlane(cloud, 100, 0.2);
    //renderPointCloud( viewer,  SegmentCloud.first, "Segmented obstacle cloud", Color(1,0,0));
    //renderPointCloud( viewer,  SegmentCloud.second, "Segmented plane cloud", Color(0,1,1));

    //typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pt_proc.Clustering(SegmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1) };

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){

        std::cout<<"cluster size ";
        pt_proc.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
        Box box = pt_proc.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
  
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock_old(viewer);
    
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

     std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
     auto streamIterator = stream.begin();
    
    /*
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
    */

     while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloud = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloud);

        streamIterator++;
        if (streamIterator == stream.end()){
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    }  
}