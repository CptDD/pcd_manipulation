#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pcd_processor/CloudFilter.h>
#include <pcd_processor/Segmentor.hpp>
#include <pcd_processor/FeatureComputer.hpp>
#include <pcd_processor/NNComputer.hpp>
#include <pcd_processor/N2.hpp>
#include <pcd_processor/DistributionComputer.hpp>
#include <pcd_processor/DistributionEntry.hpp>
#include <pcd_processor/Util.hpp>

using namespace std;


void read_sampled_files(boost::filesystem::path path,vector<string> &cloud_files)
{
	if(!boost::filesystem::exists(path) || !boost::filesystem::is_directory(path))
	{
		cout<<"The path does not exist!"<<endl;
		return ;
	}

	vector<string> tokens;


	boost::filesystem::directory_iterator it(path);

	for(;it!=boost::filesystem::directory_iterator();++it)
	{
		tokens.clear();
		stringstream ss;
		ss<<it->path();
		string temp_file=ss.str();

		boost::replace_all(temp_file,"\"", "");

		//cout<<"Temp file :"<<temp_file<<endl;


		boost::split(tokens,temp_file,boost::is_any_of("/"));


		if(boost::contains(tokens[tokens.size()-1],"_s.pcd"))
		{
			//cout<<tokens[tokens.size()-1]<<endl;
			cloud_files.push_back(temp_file);
		}
	}
}



void read_folders(boost::filesystem::path path,vector<string> &files)
{
	if(!boost::filesystem::exists(path) || !boost::filesystem::is_directory(path))
	{
		cout<<"The path does not exist!"<<endl;
		return ;
	}

	boost::filesystem::directory_iterator it(path);

	for(;it!=boost::filesystem::directory_iterator();++it)
	{
		stringstream ss;
		ss<<it->path()<<"/";
		string temp_file=ss.str();

		boost::replace_all(temp_file,"\"", "");

		files.push_back(temp_file);
	}
}

void set_type(pair<string,int> &info)
{
	if(boost::contains(info.first,"elongated")==1)
	{
		info.second=0;
	}else if(boost::contains(info.first,"liv")==1)
	{
		info.second=1;
	}else	if(boost::contains(info.first,"mush")==1)
	{
		info.second=2;
	}else if(boost::contains(info.first,"stan")==1)
	{
		info.second=3;
	}


}


void read_cloud(string path,pair<pair<string,int>,pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud)
{
	pcl::PCDReader reader;

	if(reader.read(path,*cloud.second)==-1)
	{
		cout<<"An error has occured during cloud reading!"<<endl;
		return;
	}

	vector<string> tokens;

	boost::split(tokens,path,boost::is_any_of("/"));
	cloud.first.first=tokens[tokens.size()-1];

}


void read_vfh_cloud(string path,pair<pair<string,int>,pcl::PointCloud<pcl::VFHSignature308>::Ptr> &vfh_cloud)
{
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if(reader.read(path,*cloud)==-1)
	{
		cout<<"An error has occured during cloud reading!"<<endl;
		return;
	}

	FeatureComputer::computeVFH(cloud,vfh_cloud.second);

	vector<string> tokens;

	boost::split(tokens,path,boost::is_any_of("/"));
	vfh_cloud.first.first=tokens[tokens.size()-1];
	set_type(vfh_cloud.first);
}



void read_clouds(string path,vector<pair<pair<string,int>,pcl::PointCloud<pcl::VFHSignature308>::Ptr> >&clouds)
{
		cout<<"Reading :"<<path<<endl;

		pair<pair<string,int>,pcl::PointCloud<pcl::VFHSignature308>::Ptr> temp_pair;
		pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud(new pcl::PointCloud<pcl::VFHSignature308>);
		temp_pair.second=cloud;

		read_vfh_cloud(path,temp_pair);
		clouds.push_back(temp_pair);
}


void recognition(vector<pair<pair<string,int>,pcl::PointCloud<pcl::VFHSignature308>::Ptr> > clouds)
{
			for(int i=0;i<clouds.size();i++)
			{
					cout<<clouds[i].first.first<<" Type :"<<clouds[i].first.second<<endl;
			}

			int most_similar=N2::search(clouds[1].second);

			cout<<most_similar<<endl;

}

void get_clouds(vector<string> folders,vector<pair<pair<string,int>,pcl::PointCloud<pcl::VFHSignature308>::Ptr> >&clouds)
{

	for(int i=0;i<folders.size();i++)
	{
		cout<<"Folder :"<<folders[i]<<endl;

		vector<string> files;
		read_sampled_files(folders[i],files);

		for(int j=0;j<files.size();j++)
		{
			read_clouds(files[j],clouds);
		}
	}
	cout<<"Read :"<<clouds.size()<<endl;

}



int main(int argc,char**argv)
{
	ros::init(argc,argv,"model_builder");

	string path=ros::package::getPath("pcd_processor");
	stringstream ss;
	vector<string> folders;
	ss<<path<<"/training/";

	boost::filesystem::path p=ss.str();

	/*read_folders(p,folders);

	//vector<pair<string,vector<pair<pair<string,int>,pair<int,int> > > > >total_dist;
	vector<pair<string,vector<DistributionEntry> > > total_dist;


	for(int i=0;i<folders.size();i++)
	{
		boost::filesystem::path temp_path=folders[i];
		vector<pair<pair<string,int>,pcl::PointCloud<pcl::VFHSignature308>::Ptr> >clouds;
		//vector<pair<pair<string,int>,pair<int,int> > >temp_dist;

		//vector<DistributionEntry> temp_distribution;
		pair<string,vector<DistributionEntry> >temp_distribution;
		temp_distribution.first=folders[i];

		vector<string> temp_folders;

		read_folders(temp_path,temp_folders);

		get_clouds(temp_folders,clouds);

		DistributionComputer::compute(clouds,temp_distribution.second);

		total_dist.push_back(temp_distribution);

		/*DistributionComputer::compute_distribution(clouds,temp_dist);

		pair<string,vector<pair<pair<string,int>,pair<int,int> > > > temp_pair;

		temp_pair.first=folders[i];
		temp_pair.second=temp_dist;

		total_dist.push_back(temp_pair);
	}*/


	/*for(int i=0;i<total_dist.size();i++)
	{
		cout<<total_dist[i].first<<endl;
		DistributionComputer::show(total_dist[i].second);
	}*/


	ss.str(string());
	ss<<path<<"/training_results/"<<"out.txt";

	/*Util::results_2_file(ss.str(),total_dist);

	vector<pair<string,vector<DistributionEntry> > > read_distribution;*/

	Util::get_results_from_file(ss.str(),read_distribution);

	for(int i=0;i<total_dist.size();i++)
	{
		cout<<read_distribution[i].first<<endl;
		DistributionComputer::show(read_distribution[i].second);
	}



	/*cout<<"Computed :"<<total_dist.size()<<endl;

	for(int i=0;i<total_dist.size();i++)
	{
		cout<<total_dist[i].first<<endl;
		DistributionComputer::show_distribution(total_dist[i].second);
	}*/


	//DistributionComputer::compute_distribution(clouds);






	return 0;
}
