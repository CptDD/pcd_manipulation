#ifndef INFORMATION_H
#define INFORMATION_H

#include <vector>

#include <despot/util/random.h>
#include <despot/util/logging.h>
#include <despot/core/history.h>

namespace despot {

class State;
class StateIndexer;
class DSPOMDP;

class Information
{
    protected:
    int class_numbers;
    DSPOMDP *model;
    Belief *pre;
    Belief *post;
    std::map<std::string,double> pdf_to_map(std::map<std::string,double> pdf);
    std::map<std::string,double> pdf_extended_to_map(std::map<std::string,double> pdf);
    std::map<std::string,double> pdf_to_tiger(std::map<std::string,double> pdf);


    public:
    Information(int classes=4):class_numbers(classes){}
    Information(DSPOMDP *m, int classes=4):model(m),class_numbers(classes){}
    void show(std::map<std::string,double> values);
    void add_pre(Belief *b);
    void add_post(Belief *b);

    std::map<std::string,double> compute_information_gain();
    std::map<std::string,double> compute_extended_information_gain();
    double compute_expected_information_gain();
    double compute_entropy();
    double get_max(Belief *b);
    double compute_extended_expected_information_gain();
};



} // namespace despot

#endif
