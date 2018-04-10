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
    DSPOMDP *model;
    Belief *pre;
    Belief *post;
    std::map<std::string,double> pdf_to_map(std::map<std::string,double> pdf);
    public:
    Information(DSPOMDP *m);
    void show();
    void add_pre(Belief *b);
    void add_post(Belief *b);

    std::map<std::string,double> compute_information_gain();
    double compute_expected_information_gain();
    double compute_entropy();
};



} // namespace despot

#endif
