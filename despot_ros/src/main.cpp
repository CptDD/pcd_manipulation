#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <despot/simple_tui.h>
#include <despot/pomdpx/pomdpx.h>


using namespace std;
using namespace despot;

class TUI: public SimpleTUI
{
public:
  TUI(){

  }

  DSPOMDP* InitializeModel(string pomdpx_file)
  {
    DSPOMDP* model=NULL;
    model=new POMDPX(pomdpx_file);

    /*if(options[E_PARAMS_FILE])
    {
      model=new POMDPX("/home/cptd/go_ws/src/despot/data/p.pomdpx");
      //model=new POMDPX(options[E_PARAMS_FILE].arg);
    }else
    {
      cout<<"An error has occured while loading the file!"<<endl;
    }*/

    return model;
  }

  void InitializeDefaultParameters(){

  }

};


void results_to_file(string filename,string content)
{
  ofstream out;
  out.open(filename);
  out<<content;
  out.close();
} 



int main(int argc, char* argv[]) {

  ros::init(argc,argv,"ros_despot_node");
  ros::NodeHandle nh;

  string path=ros::package::getPath("despot");
  stringstream output_filename;
  output_filename<<path<<"/results/results.txt";

  cout<<output_filename.str()<<endl;

  string pomdpx_file="/home/cptd/go_ws/src/despot/data/p.pomdpx";

  string content=TUI().run(pomdpx_file);

  results_to_file(output_filename.str(),content);

  return 0;
}
