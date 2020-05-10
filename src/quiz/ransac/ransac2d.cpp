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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	int numOutliers = cloud->size();
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	for (int it = 0; it < maxIterations; ++it)
	{
		int indexP1, indexP2;
		indexP1 = rand() % cloud->size();
		do
			indexP2 = rand() % cloud->size();
		while (indexP1 == indexP2);

		float a = (*cloud)[indexP1].y - (*cloud)[indexP2].y;
		float b = (*cloud)[indexP2].x - (*cloud)[indexP1].x;
		float c = ((*cloud)[indexP1].x * (*cloud)[indexP2].y) -
				  ((*cloud)[indexP2].x * (*cloud)[indexP1].y);
		
		std::unordered_set<int> newInliers;
		int index = 0;
		int newNumOutliers = 0;
		for (auto cIt = cloud->begin(); cIt != cloud->end(); ++cIt)
		{
			float d = fabs(a*cIt->x + b*cIt->y + c) / sqrt(a*a + b*b);
			if (d < distanceTol)
			{
				newInliers.insert(index);
			}
			else
			{
				newNumOutliers++;
				if (newNumOutliers > numOutliers)
					break;
			}
			index++;
		}
		
		if (newInliers.size() > inliersResult.size())
		{	
			inliersResult = newInliers;
			numOutliers = newNumOutliers;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " microseconds" << std::endl;
	
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	int numOutliers = cloud->size();
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	for (int it = 0; it < maxIterations; ++it)
	{
		std::unordered_set<int> newInliers;
		while (newInliers.size() < 3)
			newInliers.insert(rand() % cloud->size());

		auto pIt = newInliers.begin();
		float x1 = (*cloud)[*pIt].x;
		float y1 = (*cloud)[*pIt].y;
		float z1 = (*cloud)[*pIt].z;
		pIt++;
		float x2 = (*cloud)[*pIt].x;
		float y2 = (*cloud)[*pIt].y;
		float z2 = (*cloud)[*pIt].z;
		pIt++;
		float x3 = (*cloud)[*pIt].x;
		float y3 = (*cloud)[*pIt].y;
		float z3 = (*cloud)[*pIt].z;

		float a = ((y2-y1)*(z3-z1)) - ((z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1)) - ((x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1)) - ((y2-y1)*(x3-x1));
		float d = -((a*x1) + (b*y1) + (c*z1));
		
		int index = 0;
		int newNumOutliers = 0;
		for (auto cIt = cloud->begin(); cIt != cloud->end(); ++cIt)
		{
			float d = fabs(a*cIt->x + b*cIt->y + c*cIt->z + d) / sqrt(a*a + b*b+ c*c);
			if (d < distanceTol)
			{
				newInliers.insert(index);
			}
			else
			{
				newNumOutliers++;
				if (newNumOutliers > numOutliers)
					break;
			}
			index++;
		}
		
		if (newInliers.size() > inliersResult.size())
		{	
			inliersResult = newInliers;
			numOutliers = newNumOutliers;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " microseconds" << std::endl;
	
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
	// std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);
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
