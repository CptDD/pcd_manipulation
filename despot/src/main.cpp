#include <ros/ros.h>
#include <iostream>
#include <despot/simple_tui.h>
#include <despot/pomdpx/pomdpx.h>


using namespace std;
using namespace despot;

class TUI: public SimpleTUI {
public:
  TUI() {
  }

  DSPOMDP* InitializeModel(option::Option* options) {
    DSPOMDP* model = NULL;
    if (options[E_PARAMS_FILE]) {
        model = new POMDPX(options[E_PARAMS_FILE].arg);
    } else {
      cerr << "ERROR: Specify a POMDPX model file name using -m!" << endl;
      exit(0);
    }
    return model;
  }
  
  void InitializeDefaultParameters() {
  }
};

int main(int argc, char* argv[]) {

  ros::init(argc,argv,"ros_despot");
  ros::NodeHandle nh;


  return TUI().run(argc, argv);
}
