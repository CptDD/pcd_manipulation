#ifndef DISTRIB_E
#define DISTRIB_E

#include <iostream>
#include <pcd_processor/N2.hpp>

using namespace std;

class DistributionEntry{

private:
  string label;
  int type;
  int total_apperances;
  int positives;
  double percentage;
public:

  DistributionEntry(string label,int type,int total_apperances,int positives,double percentage)
  {
    this->label=label;
    this->type=type;
    this->total_apperances=total_apperances;
    this->positives=positives;
    this->percentage=percentage;
  }

  DistributionEntry()
  {
    label="default";
    type=0;
    total_apperances=0;
    positives=0;
    percentage=0.0;
  }

  void set_label(string label)
  {
    this->label=label;
  }

  void set_type(int type)
  {
    this->type=type;
  }

  void set_total(int total)
  {
    this->total_apperances=total;
  }
  void set_positives(int pos)
  {
    this->positives=pos;
  }
  void set_percentage(double percentage)
  {
    this->percentage=percentage;
  }



  int get_total()
  {
    return total_apperances;
  }

  int get_type()
  {
    return type;
  }

  void plus_total()
  {
    total_apperances++;
  }

  void plus_positives()
  {
    positives++;
  }

  int get_positives()
  {
    return positives;
  }

  double get_percentage()
  {
    return percentage;
  }

  string get_label()
  {
    return label;
  }

  void compute_percentage()
  {
    double perc=(double)positives/total_apperances;
    //perc*=100;
    cout<<"Percentage :"<<perc<<endl;
      this->set_percentage(perc);
  }

  void show()
  {
     cout<<"Label:"<<label<<"\t"<<"Type:"<<type<<"\t"<<"Good:"<<positives<<"\t"<<"How many:"
     <<total_apperances<<"\t"<<"Percentage:"<<percentage<<endl;
  }
};

#endif
