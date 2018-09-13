#ifndef SEGMENTOR
#define SEGMENTOR

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
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/uniform_sampling.h>


#define Z_MIN 0.075
#define Z_MAX 0.2
#define X_MIN 0.75
#define X_MAX 0.82

#define Y_MIN_1 0.35
#define Y_MAX_1 0.45

#define Y_MIN_2 0.2
#define Y_MAX_2 0.35

#define Z_MIN_REAL 0.01
#define Z_MAX_REAL 0.1
#define X_MIN_REAL 0.55
#define X_MAX_REAL 0.95
#define Y_MIN_REAL -0.2
#define Y_MAX_REAL -0.1

using namespace std;

class Segmentor
{
public:

	static void pass_filter_bulb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("z");
		pass_filter.setFilterLimits(Z_MIN,Z_MAX);
		pass_filter.filter(*filtered_cloud);

		pass_filter.setInputCloud(filtered_cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(X_MIN,X_MAX);
		pass_filter.filter(*filtered_cloud);

	}


	static void pass_filter_bulb_principal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("z");
		pass_filter.setFilterLimits(Z_MIN,Z_MAX);
		pass_filter.filter(*filtered_cloud);

		pass_filter.setInputCloud(filtered_cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(X_MIN,X_MAX);
		pass_filter.filter(*filtered_cloud);


		pass_filter.setInputCloud(filtered_cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(Y_MIN_1,Y_MAX_1);
		pass_filter.filter(*filtered_cloud);

	}

	static void pass_filter_bulb_secondary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("z");
		pass_filter.setFilterLimits(Z_MIN,Z_MAX);
		pass_filter.filter(*filtered_cloud);

		pass_filter.setInputCloud(filtered_cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(X_MIN,X_MAX);
		pass_filter.filter(*filtered_cloud);


		pass_filter.setInputCloud(filtered_cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(Y_MIN_2,Y_MAX_2);
		pass_filter.filter(*filtered_cloud);

	}



	static void pass_filter_bulb_real(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
	{
		pcl::PassThrough<pcl::PointXYZ> pass_filter;
		pass_filter.setInputCloud(cloud);
		pass_filter.setFilterFieldName("z");
		pass_filter.setFilterLimits(Z_MIN_REAL,Z_MAX_REAL);
		pass_filter.filter(*filtered_cloud);

		pass_filter.setInputCloud(filtered_cloud);
		pass_filter.setFilterFieldName("x");
		pass_filter.setFilterLimits(X_MIN_REAL,X_MAX_REAL);
		pass_filter.filter(*filtered_cloud);


		pass_filter.setInputCloud(filtered_cloud);
		pass_filter.setFilterFieldName("y");
		pass_filter.setFilterLimits(Y_MIN_REAL,Y_MAX_REAL);
		pass_filter.filter(*filtered_cloud);

	}


	static void transform_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		double thetha=M_PI/2;

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();

		transform.rotate (Eigen::AngleAxisf (thetha, Eigen::Vector3f::UnitX()));
		pcl::transformPointCloud(*cloud, *cloud, transform);

	}

	static void segment_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

  		seg.setOptimizeCoefficients (true);
  		seg.setModelType (pcl::SACMODEL_PLANE);
  		seg.setMethodType (pcl::SAC_RANSAC);
  		seg.setMaxIterations (100);
  		seg.setDistanceThreshold (0.02);

  		seg.setInputCloud (cloud);
    	seg.segment (*inliers, *coefficients);

    	pcl::ExtractIndices<pcl::PointXYZ> extract;
    	extract.setInputCloud (cloud);
    	extract.setIndices (inliers);
    	extract.setNegative (false);

    	extract.filter (*cloud_plane);
    	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    	extract.setNegative (true);
    	extract.filter (*cloud_filtered);

		pcl::copyPointCloud(*cloud_filtered,*cloud);    	
	}

	static vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extract_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{

		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > output_clouds;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  		tree->setInputCloud (cloud);

  		vector<pcl::PointIndices> cluster_indices;
  		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	
		ec.setClusterTolerance (0.01); // 1cm
  		ec.setMinClusterSize (100);
  		ec.setMaxClusterSize (25000);
  		ec.setSearchMethod (tree);
  		ec.setInputCloud (cloud);
	  	ec.extract (cluster_indices);

  		int j = 0;
  		for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  	{
    		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    		for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    		{
     	 		cloud_cluster->points.push_back (cloud->points[*pit]); 
    		}	
    		cloud_cluster->width = cloud_cluster->points.size ();
    		cloud_cluster->height = 1;
    		cloud_cluster->is_dense = true;

    		cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    		
    		if(cloud_cluster->points.size()>10)
    		{
	    		output_clouds.push_back(cloud_cluster);
    		}
    		j++;
  		}

  		return output_clouds;

	}

	static void sample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::UniformSampling<pcl::PointXYZ> sampler;
		pcl::PointCloud<int> keypointIndices;

		sampler.setInputCloud(cloud);
		sampler.setRadiusSearch(0.01);
		//sampler.compute(keypointIndices);
		sampler.filter(*cloud_filtered);
		
		cout<<"Cloud sampled with :"<<cloud_filtered->points.size()<<endl;
		
		pcl::copyPointCloud(*cloud_filtered,*cloud);		

		/*pcl::copyPointCloud(*cloud, keypointIndices.points, *cloud_filtered);


		cout<<"Sampled :"<<cloud_filtered->points.size()<<endl;

		if(cloud->points.size()>10)
		{
			pcl::copyPointCloud(*cloud_filtered,*cloud);
		}*/


	}


};

#endif
