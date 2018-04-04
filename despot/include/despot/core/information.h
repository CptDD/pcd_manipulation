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
    public:
    Information(DSPOMDP *m);
    void show();
    void add_pre(Belief *b);
    void add_post(Belief *b);

    void compute_information_gain();
};



} // namespace despot

#endif
