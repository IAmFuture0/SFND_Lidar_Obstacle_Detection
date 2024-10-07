/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){
  	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int lowerbound = 0;
  	int upperbound = cloud->points.size()-1;
  
	// For max iterations 
	for(int i = 0; i < maxIterations; i++){
      int random_index1 = lowerbound + std::rand() % (upperbound - lowerbound + 1);
      int random_index2 = lowerbound + std::rand() % (upperbound - lowerbound + 1);
      int random_index3 = lowerbound + std::rand() % (upperbound - lowerbound + 1);   
      
      // Randomly sample subset and fit line
      double x1 = cloud->points[random_index1].x;
      double y1 = cloud->points[random_index1].y;
      double z1 = cloud->points[random_index1].z;
      double x2 = cloud->points[random_index2].x;
      double y2 = cloud->points[random_index2].y;
      double z2 = cloud->points[random_index2].z;
	  double x3 = cloud->points[random_index3].x;
      double y3 = cloud->points[random_index3].y;
      double z3 = cloud->points[random_index3].z;
      
      double v1_x = x2-x1, v1_y = y2-y1, v1_z = z2-z1;
      double v2_x = x3-x1, v2_y = y3-y1, v2_z = z3-z1;
      
      double coeffA = v1_y*v2_z - v1_z*v2_y, coeffB = v1_z*v2_x - v1_x*v2_z, coeffC = v1_x*v2_y -v1_y*v2_x;
      double coeffD = -1*(coeffA*x1 + coeffB*y1 + coeffC*z1);
      
      // Measure distance between every point and fitted line
      std::unordered_set<int> tmp_inliers;
      
      for(int i = 0; i < cloud->points.size(); i++){
        double pointX = cloud->points[i].x;
        double pointY = cloud->points[i].y;
        double pointZ = cloud->points[i].z;

        double distance = fabs(coeffA * pointX + coeffB * pointY + coeffC * pointZ + coeffD) / sqrt(coeffA*coeffA + coeffB*coeffB + coeffC*coeffC);
        // If distance is smaller than threshold count it as inlier
        if (distance < distanceTol){
        	tmp_inliers.insert(i);
        }
      }
      if(tmp_inliers.size() > inliersResult.size()){
        inliersResult = tmp_inliers;
      }
    }
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int lowerbound = 0;
  	int upperbound = cloud->points.size()-1;
  
	// For max iterations 
	for(int i = 0; i < maxIterations; i++){
      int random_index1 = lowerbound + std::rand() % (upperbound - lowerbound + 1);
      int random_index2 = lowerbound + std::rand() % (upperbound - lowerbound + 1);
      
	// Randomly sample subset and fit line
      double x1 = cloud->points[random_index1].x;
      double y1 = cloud->points[random_index1].y;
      double x2 = cloud->points[random_index2].x;
      double y2 = cloud->points[random_index2].y;
	  double  coeffA = y2 - y1;
      double  coeffB = x1 - x2;
      double  coeffC = x2*y1 - x1*y2;
      
	// Measure distance between every point and fitted line
      std::unordered_set<int> tmp_inliers;
      
      for(int i = 0; i < cloud->points.size(); i++){
        double pointX = cloud->points[i].x;
        double pointY = cloud->points[i].y;
        double pointZ = cloud->points[i].z;
		
        double distance = fabs(coeffA * pointX + coeffB * pointY + coeffC) / sqrt(coeffA*coeffA + coeffB*coeffB);
        // If distance is smaller than threshold count it as inlier
        if (distance < distanceTol){
        	tmp_inliers.insert(i);
        }
      }
      if(tmp_inliers.size() > inliersResult.size()){
        inliersResult = tmp_inliers;
      }
    }
	// Return indicies of inliers from fitted line with most inliers
    
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 10, 1);
  	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
