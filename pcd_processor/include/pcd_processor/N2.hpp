#ifndef NEAREST2
#define NEAREST2

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

typedef std::pair<std::string, std::vector<float> > vfh_model;

class N2{
public:

  static bool loadHist(pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud,vfh_model &vfh)
  {
      vfh.first="test_file";

      vfh.second.resize (308);



      for (size_t i = 0; i <308; ++i)
      {
        vfh.second[i] = cloud->points[0].histogram[i];
      }


      return true;
  }

  static bool loadHist(const boost::filesystem::path &path, vfh_model &vfh)
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


  static inline void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model,
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

  static bool loadFileList (std::vector<vfh_model> &models, const std::string &filename)
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

  static int search(pcl::PointCloud<pcl::VFHSignature308>::Ptr cloud)
  {
    int k = 4;

    double thresh = 100;

    string s_index="/home/cptd/test_pcl/pcd_models_new/liv_s_vfh.pcd";

    vfh_model histogram;


    /*if (!loadHist (s_index, histogram))
    {
      cout<<"Could not load test!"<<endl;
      return ;
    }*/

    loadHist(cloud,histogram);

    std::string kdtree_idx_file_name = "/home/cptd/test_pcl/kdtree.idx";
    std::string training_data_h5_file_name = "/home/cptd/test_pcl/training_data.h5";
    std::string training_data_list_file_name = "/home/cptd/test_pcl/training_data.list";

    std::vector<vfh_model> models;
    flann::Matrix<int> k_indices;
    flann::Matrix<float> k_distances;
    flann::Matrix<float> data;
    // Check if the data has already been saved to disk
    if (!boost::filesystem::exists (training_data_h5_file_name.c_str()) || !boost::filesystem::exists (training_data_list_file_name.c_str()))
    {
      pcl::console::print_error ("Could not find training data models files %s and %s!\n",
          training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
      return -1;
    }
    else
    {
      loadFileList (models, training_data_list_file_name);
      flann::load_from_file (data, training_data_h5_file_name, "training_data");
      /*pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n",
          (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());*/
    }

    // Check if the tree index has already been saved to disk
    if (!boost::filesystem::exists (kdtree_idx_file_name))
    {
      pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
      return -1;
    }
    else
    {
      flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (kdtree_idx_file_name.c_str()));
      index.buildIndex ();
      nearestKSearch (index, histogram, k, k_indices, k_distances);
    }

    //cout<<"Most similar :"<<k_indices[0][0]<<endl;

    // Output the results on screen
    /*for (int i = 0; i < k; ++i)
    {
      pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n",
          i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
    }*/

    return k_indices[0][0];
  }




};



#endif
