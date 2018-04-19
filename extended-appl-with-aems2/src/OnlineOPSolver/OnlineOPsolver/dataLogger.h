#pragma once
#include <vector>
#include <list>
#include "Const.h"
#include "FacmodelStructs.h"
#include "MObject.h"
#include "MOMDP.h"
// for data logging
#include <iostream>
#include <fstream>
#include <iomanip>


using namespace std;
using namespace momdp;

struct ActionObservation
{
	int action, observation, nextState;
	double reward;
};

class dataLogger
{
private:
	list<SharedPointer<BeliefWithState> > Belief;
	list<SharedPointer<BeliefWithState> > realState;
	list<ActionObservation> decisions;
	SharedPointer<MOMDP> problem;

	string outputFileName;
	ofstream streamOut;

public:

	
	void setup(SharedPointer<MOMDP> problem, string fileName);
	double calculateReward(double gamma);
	void addBelief(SharedPointer<BeliefWithState> s);
	void addRealState(SharedPointer<BeliefWithState> s);
	void addTuple(int action, int observation, int Xn, double reward);
	void addData(SharedPointer<BeliefWithState> belief, SharedPointer<BeliefWithState> actualState, int action, int observation, int Xn, double reward);

	void printTuple(map<string, string> tuple);
	bool printToFile(void);
	void printToScreen(void);

	dataLogger();
~dataLogger();
};

