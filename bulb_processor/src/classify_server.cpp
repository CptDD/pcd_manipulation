#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Vector3.h>

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

#define SERVICE_NAME "bulb_processor_service"

using namespace std;
using namespace boost::filesystem;





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
		//Segmentor::sample_cloud(clouds[i]);

		pcl::PointCloud<pcl::VFHSignature308>::Ptr temp_vfh(new pcl::PointCloud<pcl::VFHSignature308>);

		FeatureComputer::computeVFH(clouds[i],temp_vfh);

		cloud_features.push_back(temp_vfh);
	}

	return cloud_features;
}


pcl::PointCloud<pcl::VFHSignature308>::Ptr compute_feature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_cloud(new pcl::PointCloud<pcl::VFHSignature308>);
	FeatureComputer::computeVFH(cloud,vfh_cloud);

	return vfh_cloud;
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

		cout<<"Reading cloud :"<<files[i]<<endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

		if(reader.read(files[i],*cloud)!=-1)
		{
			cout<<"Read cloud :"<<files[i]<<endl;
			cout<<"Cloud has :"<<cloud->points.size()<<" points!"<<endl;
			
			pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> temp_pair;

			temp_pair.first=temp_file;
			temp_pair.second=cloud;

			clouds.push_back(temp_pair);

		}else
		{
			cout<<"Error reading cloud for :"<<files[i]<<endl;
		}

	}

	return clouds;
}

void show_results(map<string,vector<int> >results)
{
	map<string,vector<int> >::iterator it;
	for(it=results.begin();it!=results.end();++it)
  	{
  		cout<<"For :"<<it->first<<endl;

  		for(int i=0;i<it->second.size();i++)
  		{
  			cout<<it->second[i]<<" ";
  		}
  		cout<<endl;
  	}
}

void write_results(map<string,vector<int> >results,string bulb_type)
{
	ofstream file;

  	const string pkg="bulb_processor";
	string path = ros::package::getPath(pkg);

 	stringstream ss;
  	ss<<path<<"/results/"<<bulb_type<<".txt";

  	file.open (ss.str());

  	map<string,vector<int> >::iterator it;

  	for(it=results.begin();it!=results.end();++it)
  	{
  		file<<"Bulb:\t"<<bulb_type<<endl;
  		file<<"Label:\t"<<it->first<<endl;

  		for(int i=0;i<it->second.size();i++)
  		{
  			double percentage=(double)it->second[i]/10;

  			switch(i)
  			{
  				case 0:
  				{
  					file<<"Percentage_e:\t"<<percentage<<endl;
  					break;
  				}

  				case 1:
  				{
  					file<<"Percentage_l:\t"<<percentage<<endl;
  					break;
  				}

  				case 2:
  				{
  					file<<"Percentage_m:\t"<<percentage<<endl;
  					break;
  				}

  				case 3:
  				{
  					file<<"Percentage_s:\t"<<percentage<<endl;
  					break;
  				}

  				default:
  				{
  					break;
  				}
  			}
  		}
  		file<<"---"<<endl;

  	}

  	file.close();
}


void classify(vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> clouds)
{
	int elongated=0;
	int livarno=0;
	int mushroom=0;
	int standard=0;

	for(int i=0;i<clouds.size();i++)
	{
		string classification_result=N2::search(clouds[i]);

		cout<<"Classification result :"<<classification_result<<endl;

		if(boost::contains(classification_result,"elongated"))
		{
			cout<<classification_result<<" elongated!"<<endl;
			elongated++;
		}else if(boost::contains(classification_result,"livarno")==1)
		{
			cout<<classification_result<<" livarno!"<<endl;
			livarno++;
		}else if(boost::contains(classification_result,"mushroom")==1)
		{
			cout<<classification_result<<" mushroom!"<<endl;
			mushroom++;
		}else if(boost::contains(classification_result,"standard")==1)
		{
			cout<<classification_result<<" standard!"<<endl;
			standard++;
		}
	}

	cout<<"Classification :"<<elongated<<" "<<livarno<<" "<<mushroom<<" "<<standard<<endl; 
}

map<string,vector<int> >prepare_results(vector<pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> >clouds)
{
	map<string,vector<int> > results;

	vector<string> tokens;
	vector<string> last_tokens;

	for(int i=0;i<clouds.size();i++)
	{
		string temp_file=clouds[i].first;
		tokens.clear();
		last_tokens.clear();

		boost::split(tokens,temp_file,boost::is_any_of("/"));

		boost::split(last_tokens,tokens[tokens.size()-1],boost::is_any_of("_"));

		string point_label=last_tokens[last_tokens.size()-2];

		if(results.find(point_label)==results.end())
		{

			vector<int> bulb_counts;
			bulb_counts.resize(4);
			results[point_label]=bulb_counts;
		}
	}

	return results;
}


map<string,vector<int> > classify_clouds(vector<pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> >clouds)
{
	vector<string> tokens;
	vector<string> last_tokens;

	map<string,vector<int> >results=prepare_results(clouds);

	for(int i=0;i<clouds.size();i++)
	{
		string temp_file=clouds[i].first;
		tokens.clear();
		last_tokens.clear();

		boost::split(tokens,temp_file,boost::is_any_of("/"));

		boost::split(last_tokens,tokens[tokens.size()-1],boost::is_any_of("_"));

		string point_label=last_tokens[last_tokens.size()-2];

		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh_cloud=compute_feature(clouds[i].second);

		string classification_result=N2::search(vfh_cloud);

		cout<<"For cloud:"<<clouds[i].first<<" point label : "<<point_label<<" result :"<<classification_result<<endl;

		map<string,vector<int> >::iterator it;

		it=results.find(point_label);

		if(it!=results.end())
		{

			if(boost::contains(classification_result,"elongated")==1)
			{
				it->second[0]++;
			}else if(boost::contains(classification_result,"livarno")==1)
			{
				it->second[1]++;
			}else if(boost::contains(classification_result,"mushroom")==1)
			{
				it->second[2]++;
			}else if(boost::contains(classification_result,"standard")==1)
			{
				it->second[3]++;
			}
		}

	}
	return results;

}

/*




bool processor_service(pcd_processor::classify::Request &req,pcd_processor::classify::Response &res)


	CoudFilter::cleanCloud(cloud);
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

}*/


int main(int argc,char**argv)
{
	ros::init(argc,argv,"bulb_processor_classification");

	ros::NodeHandle nh;

	//ros::ServiceServer server=nh.advertiseService(SERVICE_NAME,);

	string pa=ros::package::getPath("pcd_cloud_saver");

	string bulb_type="standard";

	stringstream ss;
	ss<<pa<<"/clouds/"<<bulb_type<<"_segmented/";

	vector<string> files=get_files(ss.str());
	vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> vfh_clouds;


	vector<pair<string,pcl::PointCloud<pcl::PointXYZ>::Ptr> >clouds;

	clouds=read_clouds_pair(files);
	map<string,vector<int> >results=classify_clouds(clouds);
	show_results(results);
	write_results(results,bulb_type);

	/*vfh_clouds=compute_features(clouds);

	cout<<"vfh_clouds"<<vfh_clouds.size()<<endl;

	classify(vfh_clouds);*/

	return 0;
}
