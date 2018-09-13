#ifndef UTIL
#define UTIL

#include <iostream>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <string>

using namespace std;

class Util{

public:
  


  static map<string,vector<double> > read_pdf(string filename)
  {
    cout<<"Reading the results from the file!"<<endl;
    map<string,vector<double> >pdf;

    ifstream in(filename.c_str());

    if(!in)
    {
      cout<<"Error while opening  file :"<<filename<<"!"<<endl;
    }

    vector<double> values;
    string label;

    while(!in.eof())
    {
      string line;
      getline(in,line);

     

      if(boost::contains(line,"---")==0 && !line.empty())
      {   
          if(boost::contains(line,"Label:")==1)
          {
            values.clear();

            vector<string> tokens;

            boost::split(tokens,line,boost::is_any_of("\t"));

            label=tokens[tokens.size()-1];
            boost::replace_all(label,"p", "");

          }

          if(boost::contains(line,"Percentage_")==1)
          {
            vector<string> tokens;

            boost::split(tokens,line,boost::is_any_of("\t"));

            stringstream ss;
            ss<<tokens[tokens.size()-1];

            double temp_value;
            ss>>temp_value;
            values.push_back(temp_value);
          }

      }else
      {
        pdf[label]=values;
      }
    }
    
    return pdf;
  }

 /* 



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

  }*/
};

#endif
