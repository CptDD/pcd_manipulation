#ifndef DISTRIB_CM
#define DISTRIB_CM

#include <iostream>
#include <pcd_processor/N2.hpp>
#include <pcd_processor/DistributionEntryMultiple.hpp>

using namespace std;

class DistributionComputerMultiple{
public:
	static void initial_setup(vector<DistributionEntryMultiple> &dist,int number_of_classes)
	{
		DistributionEntryMultiple d1,d2,d3,d4;

		d1.set_label("elongated");
      	d1.set_type(0);
      	d1.set_number_of_classes(number_of_classes);

      	d2.set_label("livarno");
      	d2.set_type(1);
      	d2.set_number_of_classes(number_of_classes);

      	d3.set_label("mushroom");
      	d3.set_type(2);
      	d3.set_number_of_classes(number_of_classes);

      	d4.set_label("standard");
      	d4.set_type(3);
      	d4.set_number_of_classes(number_of_classes);

      	dist.push_back(d1);
      	dist.push_back(d2);
      	dist.push_back(d3);
      	dist.push_back(d4);
	}


	static void compute(vector<pair<pair<string,int>,pcl::PointCloud<pcl::VFHSignature308>::Ptr> > clouds,
  	vector<DistributionEntryMultiple> &dist,int number_of_classes)
  	{
      DistributionComputerMultiple::initial_setup(dist,number_of_classes);

      for(int i=0;i<clouds.size();i++)
      {

      	dist[clouds[i].first.second].plus_total();
      	int most_similar2=N2::search(clouds[i].second);

      	cout<<"Searched for :"<<clouds[i].first.first<<" "<<clouds[i].first.second<<" and got :"<<most_similar2<<endl;
 		dist[clouds[i].first.second].plus_positives(most_similar2);

      }

      
      DistributionComputerMultiple::update_percentages(dist);

     
     	
  	}

  	static void update_percentages(vector<DistributionEntryMultiple> &dist)
  	{
  		for(int i=0;i<dist.size();i++)
  		{
  			dist[i].update_percentage();
  		}
  	}


	static void show(vector<DistributionEntryMultiple> dist)
  	{
    	cout<<"======================="<<endl;
    	for(int i=0;i<dist.size();i++)
    	{
        	dist[i].show();
    	}
    	cout<<"======================="<<endl;
  }

	

};
#endif