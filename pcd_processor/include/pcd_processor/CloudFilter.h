#ifndef CLOUD_FILTER
#define CLOUD_FILTER

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>

class CloudFilter
{
public:

	static void cleanCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		std::cout<<"===CLEANING THE CLOUD==="<<std::endl;
		std::vector<int> indices;
		std::cout<<"Before :"<<cloud->points.size()<<std::endl;
		pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);
		std::cout<<"After  :"<<cloud->points.size()<<std::endl;
		std::cout<<"========================"<<std::endl;

	}

	static void computeCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		std::cout<<"===COMPUTING THE CENTROID==="<<std::endl;
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud,centroid);
		std::cout<<"The centroid is :"<<centroid.x()<<" "<<centroid.y()<<" "<<centroid.z()<<std::endl;
		std::cout<<"========================"<<std::endl;
	}
};

#endif