#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <bulb_processor/CloudFilter.h>
#include <bulb_processor/Segmentor.hpp>


#define SERVICE_NAME "bulb_processor"

using namespace std;
using namespace boost::filesystem;


void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,string cloud_name,string cloud_type)
{
	if(cloud->points.size()!=0)
	{
		pcl::PCDWriter writer;

		cout<<"Saving the cloud!"<<endl;

		const string pkg="pcd_cloud_saver";
		string path = ros::package::getPath(pkg);

		//cout<<"Path :"<<path<<endl;

		stringstream ss;
		ss<<path<<"/clouds/"<<cloud_type<<"_segmented/"<<cloud_name;

		writer.write(ss.str(),*cloud);
	}else
	{
		cout<<"Cloud is empty!"<<endl;
	}

}



void save_clouds(vector<pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> > clouds,string cloud_type)
{
	stringstream ss;

	for(int i=0;i<clouds.size();i++)
	{
		ss.str(string());
		ss<<clouds[i].first;
		Segmentor::sample_cloud(clouds[i].second);
		save_cloud(clouds[i].second,ss.str(),cloud_type);
	}
}

void save_clouds(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds,string cloud_type)
{
	stringstream ss;

	for(int i=0;i<clouds.size();i++)
	{
		ss.str(string());
		ss<<"cloud_"<<i;
		Segmentor::sample_cloud(clouds[i]);
		save_cloud(clouds[i],ss.str(),cloud_type);
	}
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


vector<string> get_files(string directory_path)
{
	path p(directory_path);

	vector<directory_entry> v;
	vector<string> files;

	if(is_directory(p))
    {
        copy(directory_iterator(p), directory_iterator(), back_inserter(v));    

       	for (vector<directory_entry>::const_iterator it=v.begin();it!=v.end();++it)
        {
        	files.push_back((*it).path().string());

        }    
    }

    return files;
}

vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >read_clouds(vector<string> files) 
{
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >clouds;

	pcl::PCDReader reader;

	for(int i=0;i<files.size();i++)
	{	
		cout<<"Reading cloud :"<<files[i]<<endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		if(reader.read(files[i],*cloud)!=-1)
		{
			cout<<"Read cloud :"<<files[i]<<endl;
			cout<<"Cloud has :"<<cloud->points.size()<<" points!"<<endl;
			clouds.push_back(cloud);
		}else
		{
			cout<<"Error reading cloud for :"<<files[i]<<endl;
		}
	}

	return clouds;
}

vector<pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> >read_clouds_pair(vector<string> files)
{
	vector<pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> > clouds;
	vector<string> tokens;
	pcl::PCDReader reader;

	for(int i=0;i<files.size();i++)
	{
		tokens.clear();
		string temp_file=files[i];


		if(boost::contains(temp_file,"_noisy_2")==1)
		{
			boost::replace_all(temp_file,"_noisy_2.pcd", ".pcd");	

		}else if(boost::contains(temp_file,"_noisy.pcd")==1)
		{
			boost::replace_all(temp_file,"_noisy.pcd", ".pcd");
		}


		boost::split(tokens,temp_file,boost::is_any_of("/"));



		cout<<"Reading cloud :"<<files[i]<<endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		if(reader.read(files[i],*cloud)!=-1)
		{
			cout<<"Read cloud :"<<files[i]<<endl;
			cout<<"Cloud has :"<<cloud->points.size()<<" points!"<<endl;
			
			pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> temp_pair;
			temp_pair.first=tokens[tokens.size()-1];
			temp_pair.second=cloud;

			clouds.push_back(temp_pair);

		}else
		{
			cout<<"Error reading cloud for :"<<files[i]<<endl;
		}

		
	}

	return clouds;
}


void segment_clouds(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr >clouds)
{
	for(int i=0;i<clouds.size();i++)
	{
		Segmentor::pass_filter_bulb_real(clouds[i],clouds[i]);
		cout<<clouds[i]->points.size()<<endl;
	}
}

void segment_clouds(vector<pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> >clouds)
{
	for(int i=0;i<clouds.size();i++)
	{
		Segmentor::pass_filter_bulb(clouds[i].second,clouds[i].second);
		cout<<clouds[i].second->points.size()<<endl;
	}

	for(int i=0;i<clouds.size();i++)
	{
		vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters=Segmentor::extract_clusters(clouds[i].second);

		for(int j=0;j<clusters.size();j++)
		{
			stringstream ss;
			if(j==0)
			{
				ss<<clouds[i].first<<"_principal.pcd";
			}else
			{
				ss<<clouds[i].first<<"_secondary.pcd";
			}

			save_cloud(clusters[j],ss.str(),"standard");
		}
	}
}






/*bool processor_service(pcd_processor::classify::Request &req,pcd_processor::classify::Response &res)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	read_cloud(cloud);

	cout<<"Cloud has :"<<cloud->points.size()<<endl;

	CloudFilter::cleanCloud(cloud);
	cout<<"Cloud has :"<<cloud->points.size()<<" points!"<<endl;
	//Segmentor::transform_cloud(cloud);

	save_cloud(cloud,"transformed");

	if(viewpoint==0)
	{
		Segmentor::pass_filter_cloud_side(cloud,cloud,0.3,0.4);
	}else if(viewpoint==1)
	{
		Segmentor::pass_filter_cloud_inter(cloud,cloud,-0.2,0.2);
	}else
	{
		Segmentor::pass_filter_cloud_top(cloud,cloud,0.3,0.5);
	}

	//Segmentor::pass_filter_cloud(cloud,cloud,-0.03,0.15);
	//Segmentor::pass_filter_cloud_side(cloud,cloud,0.3,0.4);

	//Segmentor::pass_filter_cloud_inter(cloud,cloud,-0.2,0.2);

	//Segmentor::pass_filter_cloud_top(cloud,cloud,);
	//Segmentor::pass_filter_cloud_top(cloud,cloud,0.3,0.5);

	cout<<"Cloud has :"<<cloud->points.size()<<" points!"<<endl;

	save_cloud(cloud,"filtered");

	//Segmentor::segment_plane(cloud);

	return true;

}*/



int main(int argc,char**argv)
{
	ros::init(argc,argv,"bulb_processor_node");
	ros::NodeHandle nh;

	string pa=ros::package::getPath("pcd_cloud_saver");

	string bulb_type="standard";

	stringstream ss;
	ss<<pa<<"/clouds/"<<bulb_type<<"_segmented/";

	vector<string> elongated=get_files(ss.str());

	vector<pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> >clouds=read_clouds_pair(elongated);

	segment_clouds(clouds);

	

	return 0;
}
