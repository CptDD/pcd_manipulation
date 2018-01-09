#ifndef NN_COMPUTER
#define NN_COMPUTER

#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/correspondence.h>


#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>

#include "boost/filesystem.hpp"


using namespace std;

typedef std::pair<std::string, std::vector<float> > vfh_model;

class NNComputer
{

public:

static void compute_pose(pcl::PointCloud<pcl::PointXYZ>::Ptr model,pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
	cout<<"Model orientation!";

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranlsations;
}

static void load_models()
{
	string path=ros::package::getPath("pcd_processor");
	stringstream ss;
	ss<<path<<"/model/";

	vector<std::string> files;
    

        boost::filesystem::path apk_path(ss.str());
        boost::filesystem::recursive_directory_iterator end;

        for (boost::filesystem::recursive_directory_iterator i(apk_path); i != end; ++i)
        {
            const boost::filesystem::path cp = (*i);
            files.push_back(cp.string());
        }

  		
}

static bool load_model(pcl::PointCloud<pcl::VFHSignature308>::Ptr model)
	{
		string path=ros::package::getPath("pcd_processor");

		stringstream ss;
		ss<<path<<"/model/"<<"model_vfh.pcd";


		pcl::PCDReader reader;

		if(reader.read(ss.str(),*model)==-1)
		{
			cout<<"Error reading the model!"<<endl;
			return false;
		}

		return true;
	}

	static void prepare_target(pcl::PointCloud<pcl::VFHSignature308>::Ptr data,flann::Matrix<float> &p)
	{
		for(int i=0;i<308;i++)
  		{
  			p[0][i]=data->points[0].histogram[i];
  		}
	}

	static vector<double> nearestKSearch(pcl::PointCloud<pcl::VFHSignature308>::Ptr model,
		vector<pcl::PointCloud<pcl::VFHSignature308>::Ptr> targets)
	{

		int k=1;

		flann::Matrix<int> k_indices;
  		flann::Matrix<float> k_distances;

  		flann::Matrix<float> data(new float[1*308],1,308);

  		for(int i=0;i<308;i++)
  		{
  			data[0][i]=model->points[0].histogram[i];
  		}
  	
 
  		//flann::Index<flann::ChiSquareDistance<float> > index(data, flann::LinearIndexParams ());
  		string path=ros::package::getPath("pcd_processor");
		stringstream ss;
		ss<<path<<"/model/kdtree.idx";

  		flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (ss.str()));
  		index.buildIndex();

  		//pcl::PointCloud<pcl::VFHSignature308>::Ptr model(new pcl::PointCloud<pcl::VFHSignature308>);

  		flann::Matrix<int> indices=flann::Matrix<int>(new int[k],1,k);
  		flann::Matrix<float> distances=flann::Matrix<float>(new float[k],1,k);

  		vector<double> computed_distances; 

  		for(int i=0;i<targets.size();i++)
  		{

  			flann::Matrix<float> p = flann::Matrix<float>(new float[1*308], 1, 308);

  			NNComputer::prepare_target(targets[i],p);

  			index.knnSearch(p,indices,distances,k,flann::SearchParams(512));

  			delete[] p.ptr ();

  			cout<<distances[0][0]<<endl;
  			cout<<indices[0][0]<<endl;

  			computed_distances.push_back(distances[0][0]);

  			//k_distances[i]=distances[0][0];
  		}

  		return computed_distances;
	}
};

#endif


/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
/*inline void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}
*/




	