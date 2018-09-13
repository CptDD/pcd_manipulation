#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Vector3.h>

#include <pcd_cloud_saver/save.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <bulb_processor/classify.h>

#include <bulb_processor/CloudFilter.h>
#include <bulb_processor/Segmentor.hpp>
#include <bulb_processor/FeatureComputer.hpp>
#include <bulb_processor/NNComputer.hpp>
#include <bulb_processor/Nearest.hpp>
#include <bulb_processor/N2.hpp>
#include <bulb_processor/Util.hpp>

#define SERVICE_NAME "bulb_classification_service"
#define CLOUD_SAVER_SERVICE_NAME "pcd_cloud_saver_service"

using namespace std;
using namespace boost::filesystem;


void save_cloud(string cloud_name)
{
	ros::NodeHandle nh;
	ros::ServiceClient serviceClient=nh.serviceClient<pcd_cloud_saver::save>(CLOUD_SAVER_SERVICE_NAME);
	
	pcd_cloud_saver::save srv;
	srv.request.cloud_name.data=cloud_name;

	if(serviceClient.call(srv))
	{
		cout<<"Cloud has been saved!"<<endl;
	}else
	{
		cout<<"Error while saving the cloud!"<<endl;
	}
}


bool read_cloud(string cloud_name,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	string path=ros::package::getPath("pcd_cloud_saver");

	stringstream ss;
	ss<<path<<"/clouds/"<<cloud_name<<"_noisy_2.pcd";

	pcl::PCDReader reader;

	if(reader.read(ss.str(),*cloud)==-1)
	{
		cout<<"An error has occured while reading the cloud!"<<endl;
		return false;
	}

	return true;
}


void segment_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
	Segmentor::pass_filter_bulb_principal(cloud,filtered_cloud);  
}


void segment_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &filtered_clouds)
{
	for(int i=0;i<2;i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr work_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		 copyPointCloud(*cloud, *work_cloud);

		if(i==0)
		{
			Segmentor::pass_filter_bulb_principal(work_cloud,temp_cloud);
		}else
		{
			Segmentor::pass_filter_bulb_secondary(work_cloud,temp_cloud);
		}

		filtered_clouds.push_back(temp_cloud);
	}
}

void sample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	Segmentor::sample_cloud(cloud);
}

int classify(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	Segmentor::sample_cloud(cloud);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_features(new pcl::PointCloud<pcl::VFHSignature308>);
	FeatureComputer::computeVFH(cloud,vfh_features);

	string classification_result=N2::search(vfh_features);

	cout<<"Classification result :"<<classification_result<<endl;

	if(boost::contains(classification_result,"elongated"))
	{
		cout<<classification_result<<" elongated!"<<endl;
		return 0;
	}else if(boost::contains(classification_result,"livarno")==1)
	{
		cout<<classification_result<<" livarno!"<<endl;
		return 1;
	}else if(boost::contains(classification_result,"mushroom")==1)
	{
		cout<<classification_result<<" mushroom!"<<endl;
		return 0;
	}else if(boost::contains(classification_result,"standard")==1)
	{
		cout<<classification_result<<" standard!"<<endl;
		return 1;
	}

}

bool classification_handle(bulb_processor::classify::Request &req,bulb_processor::classify::Response &res)
{
	string cloud_name="target_cloud";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_clouds;


	save_cloud(cloud_name);
	read_cloud(cloud_name,cloud);
	segment_cloud(cloud,filtered_cloud);
	sample_cloud(filtered_cloud);
			
	res.type.data=classify(filtered_cloud);

	return true;
}




int main(int argc,char**argv)
{
	ros::init(argc,argv,"bulb_processor_classification");
	ros::NodeHandle nh;

	ros::ServiceServer server=nh.advertiseService(SERVICE_NAME,classification_handle);
	cout<<"Bulb classification server is up . . ."<<endl;
	ros::spin();

	return 0;
}
