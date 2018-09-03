#ifndef UTIL
#define UTIL

#include <iostream>
#include <pcd_processor/N2.hpp>
#include <pcd_processor/DistributionEntry.hpp>
#include <pcd_processor/DistributionEntryMultiple.hpp>
#include <pcd_processor/DistributionComputer.hpp>
#include <pcd_processor/DistributionComputerMultiple.hpp>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <string>

using namespace std;

class Util{



public:
  static void results_2_file(string file_name,vector<pair<string,vector<DistributionEntry> > > dist)
  {
      ofstream out;
      out.open(file_name.c_str());


      cout<<"Writing results to file!"<<endl;

      if(!out)
      {
        cout<<"An error has occured while opening the file!"<<endl;
        return;
      }

      for(int i=0;i<dist.size();i++)
      {
        out<<dist[i].first<<endl;

        for(int j=0;j<dist[i].second.size();j++)
        {
          out<<dist[i].second[j].get_label()<<"\t"<<dist[i].second[j].get_type()<<
          "\t"<<dist[i].second[j].get_positives()<<"\t"<<dist[i].second[j].get_total()<<"\t"
          <<dist[i].second[j].get_percentage()<<endl;
        }
        
        out<<"==="<<endl;
      }

      out.close();
  }

  static void results_2_file(string file_name,vector<pair<string,vector<DistributionEntryMultiple> > > dist)
  {
     ofstream out;
     out.open(file_name.c_str());

     cout<<"Writing results to file!"<<endl;

      if(!out)
      {
        cout<<"An error has occured while opening the file!"<<endl;
        return;
      }

      for(int i=0;i<dist.size();i++)
      {
        out<<dist[i].first<<endl;

        for(int j=0;j<dist[i].second.size();j++)
        {
          out<<dist[i].second[j].get_label()<<"\t"<<dist[i].second[j].get_type()<<"\t"<<dist[i].second[j].get_total()<<"\t";

          vector<int> totals=dist[i].second[j].get_positives();
          vector<double> percentages=dist[i].second[j].get_percentages();

          for(int k=0;k<totals.size();k++)
          {
            out<<totals[k]<<"\t";
          }

          for(int k=0;k<totals.size();k++)
          {
            out<<percentages[k]<<"\t";
          }
          out<<endl;
        }
        out<<"==="<<endl;



      }

      out.close();
  }



  static void get_results_from_file(string file_name,vector<pair<string,vector<DistributionEntry> > > &dist)
  {
    cout<<"Reading results from file!"<<endl;

    //vector<pair<string,vector<DistributionEntry> > > dist;

    ifstream in(file_name.c_str());

    if(!in)
    {
      cout<<"Error while opening the file!"<<endl;
      return;
    }

    int line_nr=0;
    
    string cloud_path;
   
    vector<DistributionEntry> entries;

    while(!in.eof())
    {
      string line;
      getline(in,line);

      if(boost::contains(line,"===")==0 && !line.empty())
      {
        if(line_nr%6==0)
        {

         cloud_path=line;
         cout<<"The folder is :"<<cloud_path<<endl;

        }else
        {
      
          vector<string> tokens;
          boost::split(tokens,line,boost::is_any_of("\t"));

          if(tokens.size()!=0)
          {


            string cloud_label=tokens[0].c_str();

            int type=atoi(tokens[1].c_str());
            int positives=atoi(tokens[2].c_str());
            int total=atoi(tokens[3].c_str());
            double percentage=atof(tokens[4].c_str());


            DistributionEntry entry;

            entry.set_label(cloud_label);
            entry.set_type(type);
            entry.set_positives(positives);
            entry.set_total(total);
            entry.set_percentage(percentage);

            entries.push_back(entry);
          }
        }

        
      }else
      {


        pair<string,vector<DistributionEntry> >temp_pair;
        temp_pair.first=cloud_path;
        temp_pair.second=entries;
        dist.push_back(temp_pair);

        entries.clear();
      }

      line_nr++;
    }

    cout<<"The distribution has :"<<dist.size()<<" entries!"<<endl;

    in.close();

  }
};

#endif
