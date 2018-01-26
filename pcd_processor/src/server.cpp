#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Vector3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcd_processor/process.h>
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

bool processor_service(pcd_processor::process::Request &req,pcd_processor::process::Response &res)
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


	save_clouds(extracted_clusters);
	read_extra_vfhs(vfhs);

	cout<<"Computed VFH features for :"<<vfhs.size()<<" clouds!"<<endl;

	for(int i=0;i<vfhs.size();i++)
        {
		Nearest::nearestKSearch(vfhs[i]);
        }



	/*vector<pair<string,pcl::PointCloud<pcl::VFHSignature308>::Ptr> >models;
	NNComputer::load_models(models);
	cout<<"Loaded :"<<models.size()<<" models!"<<endl;

	save_cloud(extracted_clusters[0],"extrac");
	NNComputer::nearestK(models,vfhs[0]);



	/*pcl::PointCloud<pcl::VFHSignature308>::Ptr model(new pcl::PointCloud<pcl::VFHSignature308>);

	NNComputer::load_models();

	NNComputer::load_model(model);

	vector<double> computed_distances=NNComputer::nearestKSearch(model,vfhs);

	double min_d;
	int min_index;

	min_element(computed_distances,min_d,min_index);

	std::cout<<"The minimum element is :"<<min_index<<std::endl;

	save_cloud(extracted_clusters[min_index],"min_cloud");

	Eigen::Vector4f centroid;

	compute_centroid(extracted_clusters[min_index],centroid);

	std::cout<<"The centroid is :"<<centroid.x()<<" "<<centroid.y()<<" "<<centroid.z()<<std::endl;

	geometry_msgs::Vector3 msg_centroid;
	msg_centroid.x=centroid.x();
	msg_centroid.y=centroid.y();
	msg_centroid.z=centroid.z();

	//res.centroid=msg_centroid;


	save_cloud(cloud,"pass");

	/*ros::NodeHandle nh;

  	ros::Publisher pub;
  	pub=nh.advertise<sensor_msgs::PointCloud2> ("clouder", 1);

    sensor_msgs::PointCloud2 msg;
           	msg.header.frame_id="virtual_camera_gazebo_optical_frame";
            msg.header.stamp = ros::Time::now();
            pcl::toROSMsg(*extracted_clusters[min_index],msg);



            while(ros::ok())
            {
          	 msg.header.stamp = ros::Time::now();
             pub.publish (msg);
             ros::spinOnce ();
            	   pub.publish(msg);
            	}*/


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
