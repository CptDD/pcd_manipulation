#ifndef H_INFORMATION
#define H_INFORMATION

#include "SparseVector.h"
#include "BeliefCache.h"

class Information
{
protected:
    int class_numbers;
    map<string,double> pre_map;
    map<string,double> post_map;

    void check_map(map<string,double> &values);

public:
    Information(int classes=4):class_numbers(classes){};
    void show();
    void go();
    void set_pre(SharedPointer<SparseVector> &sp);
    void set_post(SharedPointer<SparseVector> &sp);
    map<string,double> compute_information_gain();
    double compute_expected_information_gain();
};

#endif