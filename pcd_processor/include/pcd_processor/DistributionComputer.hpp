#ifndef DISTRIB_C
#define DISTRIB_C

#include <iostream>
#include <pcd_processor/N2.hpp>
#include <pcd_processor/DistributionEntry.hpp>

using namespace std;

class DistributionComputer{


public:

  static void initial_setup(vector<DistributionEntry> &dist)
  {
      DistributionEntry d1,d2,d3,d4;

      d1.set_label("elongated");
      d1.set_type(0);

      d2.set_label("livarno");
      d2.set_type(1);

      d3.set_label("mushroom");
      d3.set_type(2);

      d4.set_label("standard");
      d4.set_type(3);

      dist.push_back(d1);
      dist.push_back(d2);
      dist.push_back(d3);
      dist.push_back(d4);
  }

  static void update_percentage(vector<DistributionEntry> &dist)
  {
    for(int i=0;i<dist.size();i++)
    {
      dist[i].compute_percentage();
    }
  }

  static void compute(vector<pair<pair<string,int>,pcl::PointCloud<pcl::VFHSignature308>::Ptr> > clouds,
  vector<DistributionEntry> &dist)
  {
      DistributionComputer::initial_setup(dist);

      for(int i=0;i<clouds.size();i++)
      {
          dist[clouds[i].first.second].plus_total();
          int most_similar2=N2::search(clouds[i].second);

          if(most_similar2==clouds[i].first.second)
          {
            dist[most_similar2].plus_positives();
          }
      }
      DistributionComputer::update_percentage(dist);
  }

  static void show(vector<DistributionEntry> dist)
  {
    cout<<"======================="<<endl;
    for(int i=0;i<dist.size();i++)
    {
        cout<<"Label:"<<dist[i].get_label()<<"\tType:"<<dist[i].get_type()<<"\tGood:"<<dist[i].get_positives()<<
        "\tTotal:"<<dist[i].get_total()<<"\tPercentage:"<<dist[i].get_percentage()<<endl;
    }
    cout<<"======================="<<endl;
  }

  static void setup(vector<pair<pair<string,int>,pair<int,int> > >&dist)
  {
    pair<pair<string,int>,pair<int,int> > temp;

    temp.first.first="elongated";
    temp.first.second=0;
    temp.second.first=0;
    temp.second.second=0;

    dist.push_back(temp);

    temp.first.first="livarno";
    temp.first.second=1;
    temp.second.first=0;
    temp.second.second=0;

    dist.push_back(temp);

    temp.first.first="mushroom";
    temp.first.second=2;
    temp.second.first=0;
    temp.second.second=0;

    dist.push_back(temp);


    temp.first.first="standard";
    temp.first.second=3;
    temp.second.first=0;
    temp.second.second=0;

    dist.push_back(temp);
  }

  static void show_distribution(vector<pair<pair<string,int>,pair<int,int> > >dist)
  {
    cout<<"====================================================================="<<endl;
    for(int i=0;i<dist.size();i++)
    {
      cout<<"Name:"<<dist[i].first.first<<"\t"<<"Type:"<<dist[i].first.second<<"\t"<<"Good:"<<dist[i].second.first<<"\t"<<
      "How many:"<<dist[i].second.second<<endl;
    }
    cout<<"====================================================================="<<endl;
  }

  static void compute_distribution(vector<pair<pair<string,int>,pcl::PointCloud<pcl::VFHSignature308>::Ptr> > clouds,
    vector<pair<pair<string,int>,pair<int,int> > >&dist)
  {

    DistributionComputer::setup(dist);

    cout<<"Computing the similarities!"<<endl;

    for(int i=0;i<clouds.size();i++)
    {
      int similar_to=N2::search(clouds[i].second);

      if(similar_to==clouds[i].first.second)
      {
        dist[clouds[i].first.second].second.first++;
      }

      dist[clouds[i].first.second].second.second++;
    }

      DistributionComputer::show_distribution(dist);
  }

};

#endif
