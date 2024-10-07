/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
/*
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

    if(renderScene){
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }
    return cars;
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
	Lidar *lidar = new Lidar(cars, 0);
  
    // Create point cloud
  	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
  	point_cloud = lidar->scan();
    
  	// Create point processor
  	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(point_cloud, 100, 0.2); 
  	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
  int clusterId = 0;
  std::vector<Color> colors = {Color{1, 0, 0}, Color{0, 1, 0}, Color{0, 0, 1}};
  for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
  	std::cout << "cluster size";
    pointProcessor.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
    
    // render Bounding box
    Box box = pointProcessor.BoundingBox(cluster);
  	renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}
*/

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

void Proximity(int index, pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, std::vector<int> &cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol){
	processed[index] = true;
  	cluster.push_back(index);
  	
  	std::vector<int> nearby = tree->search(pointCloud->points[index], distanceTol);
  	
    for(int i = 0; i < nearby.size(); i++){
      if(processed[nearby[i]] == false){
        Proximity(nearby[i], pointCloud, cluster, processed, tree, distanceTol);
      }
    }
    
}

std::vector<std::vector<int>> euclideanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, KdTree* tree, float distanceTol){
  std::vector<bool> processed(pointCloud->points.size(), false);
  std::vector<std::vector<int>> clusters;
  
  for(int i = 0; i < pointCloud->points.size(); i++){
  	if(processed[i] == true) continue;
    else{
    	std::vector<int> cluster;
      	Proximity(i, pointCloud, cluster, processed, tree, distanceTol);
      	clusters.push_back(cluster);
    }
  }
  
  return clusters;
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  // Filtering
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud (new pcl::PointCloud<pcl::PointXYZI>());
  filterCloud = pointProcessorI->FilterCloud(inputCloud, 0, Eigen::Vector4f(-20, -5, -5, 1), Eigen::Vector4f(20, 5, 5, 1));
  
  // Segmentation
  std::unordered_set<int> inliers = pointProcessorI->RansacPlane(filterCloud, 100, 0.2);
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr groundCloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacleCloud(new pcl::PointCloud<pcl::PointXYZI>());
  for(int index = 0; index < filterCloud->points.size(); index++){
  	pcl::PointXYZI point = filterCloud->points[index];
    if(inliers.count(index))
      groundCloud->points.push_back(point);
    else
      obstacleCloud->points.push_back(point);
  }
  if(inliers.size()) {
     renderPointCloud(viewer, groundCloud, "ground", Color(0, 1, 0));
     // renderPointCloud(viewer, obstacleCloud, "obstacle", Color(1, 0, 0));
  } else{
    // renderPointCloud(viewer, filterCloud, "inputCloud");
  }
  
  // Clustering
  // 1. create a KD tree for obstacle
  KdTree* tree = new KdTree;
  for (int i = 0; i < obstacleCloud->points.size(); i++){
    tree->insert(obstacleCloud->points[i], i);
  }
  
  // 2. euclidean clustering
  std::vector<std::vector<int>> clusters = euclideanCluster(obstacleCloud, tree, 0.3);
  
  // Render clusters
  int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters){
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: cluster){
          pcl::PointXYZI point;
          point.x = obstacleCloud->points[indice].x;
          point.y = obstacleCloud->points[indice].y;
          point.z = obstacleCloud->points[indice].z;
          point.intensity = obstacleCloud->points[indice].intensity;
          clusterCloud->points.push_back(point);
        }
      	renderPointCloud(viewer, clusterCloud, "cluster"+std::to_string(clusterId),colors[clusterId%3]);
     	 
      	// 3. Create Bounding Box
  		Box box = pointProcessorI->BoundingBox(clusterCloud);
        renderBox(viewer,box,clusterId);
  		
  		++clusterId;
  	}
  
  
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
  
  	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  	std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();
      
      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);
      streamIterator++;
      if(streamIterator == stream.end()) streamIterator = stream.begin();
      
      viewer->spinOnce ();
    } 
}