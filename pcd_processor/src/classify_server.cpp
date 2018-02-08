#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Vector3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcd_processor/classify.h>
#include <pcd_processor/CloudFilter.h>
#include <pcd_processor/Segmentor.hpp>
#include <pcd_processor/FeatureComputer.hpp>
#include <pcd_processor/NNComputer.hpp>
#include <pcd_processor/Nearest.hpp>
#include <pcd_processor/N2.hpp>

#include <sensor_msgs/PointCloud2.h>


#define SERVICE_NAME "pcd_processor_service"



using namespace std;


void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,string cloud_name)
{
	if(cloud->points.size()!=0)
	{
		pcl::PCDWriter writer;

		cout<<"Saving the cloud!"<<endl;

		const string pkg="pcd_processor";
		string path = ros::package::getPath(pkg);

		//cout<<"Path :"<<path<<endl;

		stringstream ss;
		ss<<path<<"/clouds/"<<cloud_name<<".pcd";

		writer.write(ss.str(),*cloud);
	}else
	{
		cout<<"Cloud is empty!"<<endl;
	}

}

void save_clouds(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds)
{
	for(int i=0;i<clouds.size();i++)
	{
		stringstream ss;
		ss<<"cluster_"<<i;
		Segmentor::sample_cloud(clouds[i]);
		save_cloud(clouds[i],ss.str());
	}
}

void read_extra_cluster(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clouds)
{
	string path=ros::package::getPath("pcd_processor");

	stringstream ss;
	ss<<path<<"/clouds/cluster_4.pcd";

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;

	if(reader.read(ss.str(),*cloud)==-1)
	{
		cout<<"Error reading the cloud!"<<endl;

		return;
	}

	clouds.push_back(cloud);
}


void read_extra_vfhs(vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> &clouds)
{
	string path=ros::package::getPath("pcd_processor");

	stringstream ss;
	ss<<path<<"/models/model_bulbs/stan_s_vfh.pcd";

	pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud(new pcl::PointCloud<pcl::VFHSignature308>);

	pcl::PCDReader reader;

	if(reader.read(ss.str(),*cloud)==-1)
	{
		cout<<"Error reading the cloud!"<<endl;

		return;
	}

	clouds.push_back(cloud);
	cout<<"Extra vfhs read :"<<clouds.size()<<endl;
}



bool read_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	string path=ros::package::getPath("pcd_cloud_saver");

	stringstream ss;
	ss<<path<<"/clouds/cloud.pcd";

	pcl::PCDReader reader;

	if(reader.read(ss.str(),*cloud)==-1)
	{
		cout<<"Error reading the cloud!"<<endl;
		return false;
	}


	return true;
}


vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> compute_features(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>clouds)
{
	vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> cloud_features;

	for(int i=0;i<clouds.size();i++)
	{
		Segmentor::sample_cloud(clouds[i]);

		pcl::PointCloud<pcl::VFHSignature308>::Ptr temp_vfh(new pcl::PointCloud<pcl::VFHSignature308>);

		FeatureComputer::computeVFH(clouds[i],temp_vfh);

		cloud_features.push_back(temp_vfh);
	}

	return cloud_features;
}

void min_element(vector<double> distances,double &min_d,int &min_index)
{
	min_index=0;
	min_d=distances[0];

	for(int i=1;i<distances.size();i++)
	{
		if(distances[i]<min_d)
		{
			min_d=distances[i];
			min_index=i;
		}
	}
}

void compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Eigen::Vector4f &centroid)
{
	pcl::compute3DCentroid(*cloud,centroid);
}

bool processor_service(pcd_processor::classify::Request &req,pcd_processor::classify::Response &res)
{


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	read_cloud(cloud);

	cout<<"Cloud has :"<<cloud->points.size()<<endl;

	CloudFilter::cleanCloud(cloud);
	cout<<"Cloud has :"<<cloud->points.size()<<" points!"<<endl;
	//Segmentor::transform_cloud(cloud);

	save_cloud(cloud,"transformed");

	Segmentor::pass_filter_cloud(cloud,cloud,-0.03,0.15);
	cout<<"Cloud has :"<<cloud->points.size()<<" points!"<<endl;

	save_cloud(cloud,"filtered");

	//Segmentor::segment_plane(cloud);

	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >extracted_clusters=Segmentor::extract_clusters(cloud);

	for(int i=0;i<extracted_clusters.size();i++)
	{
		stringstream ss;
		ss<<"original_"<<i;
		save_cloud(extracted_clusters[i],ss.str());
	}

	cout<<"Extracted :"<<extracted_clusters.size()<<endl;

	//save_clouds(extracted_clusters);


	vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> vfhs=compute_features(extracted_clusters);


	//save_clouds(extracted_clusters);
	//read_extra_vfhs(vfhs);

	cout<<"Computed VFH features for :"<<vfhs.size()<<" clouds!"<<endl;

	int result=-1;

	for(int i=0;i<vfhs.size();i++)
    {
		result=N2::search(vfhs[i]);
    }

    res.type.data=result;



	return true;

}



int main(int argc,char**argv)
{
	ros::init(argc,argv,"pcd_processor");

	ros::NodeHandle nh;

	ros::ServiceServer server=nh.advertiseService(SERVICE_NAME,processor_service);

	ROS_INFO("Point Cloud processing service up . . .");
	ros::spin();

	return 0;
}
