#ifndef FEATURE_COMPUTER
#define FEATURE_COMPUTER

#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

using namespace std;

class FeatureComputer
{

public:

	static void compute_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::Normal>::Ptr normals)
	{
		cout<<"Computing the normals!"<<endl;

		pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> normal_estimator;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

		normal_estimator.setInputCloud(cloud);
  		normal_estimator.setSearchMethod (tree);
  		normal_estimator.setRadiusSearch(0.01);
  		normal_estimator.compute (*normals);

	}

	static void computeVFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);


		FeatureComputer::compute_normals(cloud,normals);

		vfh.setInputCloud(cloud);
		vfh.setInputNormals(normals);
		vfh.setSearchMethod (tree);
		vfh.compute (*vfhs);
	}	
};

#endif