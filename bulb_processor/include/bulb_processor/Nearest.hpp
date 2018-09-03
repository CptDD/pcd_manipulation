#ifndef NEAREST
#define NEAREST

#include <iostream>
#include <ros/package.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>

using namespace std;

typedef std::pair<std::string, std::vector<float> > vfh_model;

/** \brief Loads an n-D histogram file as a VFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
  */


class Nearest{


public:
static bool
loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
  int vfh_idx;
  // Load the file as a PCD
  try
  {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type; unsigned int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    vfh_idx = pcl::getFieldIndex (cloud, "vfh");
    if (vfh_idx == -1)
      return (false);
    if ((int)cloud.width * cloud.height != 1)
      return (false);
  }
  catch (const pcl::InvalidConversionException&)
  {
    return (false);
  }

  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile (path.string (), point);
  vfh.second.resize (308);

  std::vector <pcl::PCLPointField> fields;
  getFieldIndex (point, "vfh", fields);

  for (size_t i = 0; i < fields[vfh_idx].count; ++i)
  {
    vfh.second[i] = point.points[0].histogram[i];
  }
  vfh.first = path.string ();
  return (true);
}


/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
static inline void
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

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
static bool
loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

static void load_test(pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud,vfh_model &histogram)
{
   histogram.second.resize (308);

   for (size_t i = 0; i < 308; ++i)
   {
    histogram.second[i] = cloud->points[0].histogram[i];
   }
   histogram.first = "test_file";
}

static void nearestKSearch(pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud)
{
  int k=4;
  double thresh=100;

  vfh_model histogram;
  Nearest::load_test(cloud,histogram);

  string path=ros::package::getPath("pcd_processor");
  stringstream ss;
  ss<<path<<"/models/model_bulbs_extra/kdtree.idx";

  std::string kdtree_idx_file_name =ss.str();
  cout<<"Kd :"<<kdtree_idx_file_name<<endl;

  ss.str(string());
  ss<<path<<"/models/model_bulbs_extra/training_data.h5";
  std::string training_data_h5_file_name = ss.str();
  cout<<"H5 :"<<training_data_h5_file_name<<endl;


  ss.str(string());
  ss<<path<<"/models/model_bulbs_extra/training_data.list";
  std::string training_data_list_file_name = ss.str();
  cout<<"Training list :"<<training_data_list_file_name<<endl;

  std::vector<vfh_model> models;
  flann::Matrix<int> k_indices;
  flann::Matrix<float> k_distances;
  flann::Matrix<float> data;

    Nearest::loadFileList (models, training_data_list_file_name);


    flann::load_from_file (data, training_data_h5_file_name, "training_data");
    pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n",
        (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());

    cout<<data.rows<<" "<<data.cols<<endl;


    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (kdtree_idx_file_name.c_str()));

    cout<<"here!"<<endl;

    index.buildIndex ();

    Nearest::nearestKSearch (index, histogram, k, k_indices, k_distances);

  for (int i = 0; i < k; ++i)
  {
    pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n",
        i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
  }
}
};

#endif
