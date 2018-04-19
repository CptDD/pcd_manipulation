#ifdef _WIN32
#include "stdafx.h" //  Windows stuff
#endif

#include "dataLogger.h"

void dataLogger::printTuple(map<string, string> tuple) {
	
		streamOut << "(";
	for (map<string, string>::iterator iter = tuple.begin(); iter != tuple.end(); )
	{
		streamOut << iter->second;
		if (++iter != tuple.end())
			streamOut << ",";
	}
	streamOut << ")" << endl;
}

void dataLogger::setup(SharedPointer<MOMDP> problem, string fileName)
{
	this->problem = problem;

	this->outputFileName = fileName;
	streamOut.open(fileName.c_str(), std::ofstream::out | std::ofstream::trunc);
	streamOut.close();
}

double dataLogger::calculateReward(double gamma)
{
	double rewardSum = 0.0;
	int k = 0;
	list<ActionObservation>::iterator iter = decisions.begin();
	
	while (iter != decisions.end())
	{
		rewardSum = rewardSum + pow(gamma, k)* iter->reward;
		k++;
		++iter;
	}

	return rewardSum;
}

void dataLogger::addBelief(SharedPointer<BeliefWithState> s)
{
	SharedPointer<BeliefWithState> newS(new BeliefWithState());

	copy(*newS->bvec, *s->bvec);
	newS->bvec->finalize();
	newS->sval = s->sval;

	this->Belief.push_back(newS);

}

void dataLogger::addRealState(SharedPointer<BeliefWithState> s)
{
	SharedPointer<BeliefWithState> newS(new BeliefWithState());

	copy(*newS->bvec, *s->bvec);
	newS->bvec->finalize();
	newS->sval = s->sval;

	this->realState.push_back(newS);
}

void dataLogger::addTuple(int action, int observation, int Xn, double reward)
{
	ActionObservation newA;
	
	newA.action = action;
	newA.nextState = Xn;
	newA.observation = observation;
	newA.reward = reward;
	
	this->decisions.push_back(newA);

}

void dataLogger::addData(SharedPointer<BeliefWithState> belief, SharedPointer<BeliefWithState> actualState, int action, int observation, int Xn, double reward)
{
	addBelief(belief);
	addRealState(actualState);
	addTuple(action, observation, Xn, reward);
}

bool dataLogger::printToFile(void)
{
	bool successful = false;
	
	//ofstream *out = this->streamOut;
	string file = this->outputFileName;
	streamOut.open(file.c_str(), ofstream::out | ofstream::app);
	list< SharedPointer<BeliefWithState> >::iterator iterBelief, iterRealState;
	list<ActionObservation>::iterator iterDecision;
	
	if (streamOut.is_open())
	{
		iterBelief = this->Belief.begin();
		iterRealState = this->realState.begin();
		iterDecision = this->decisions.begin();

		streamOut << ">>> begin\n";

		while (iterBelief != this->Belief.end() || iterRealState != this->realState.end() || iterDecision != this->decisions.end())
		{
			//actual X state, X might be a distribution at first time step
			map<string, string> obsState = problem->getFactoredObservedStatesSymbols(iterRealState->get()->sval);
			if (obsState.size() > 0) {
				streamOut.width(4);
				streamOut << left << "X" << ":";
				printTuple(obsState);
			}

			//actual Y state
			streamOut.width(4);
			streamOut << left << "Y" << ":";
			int yIndex = iterRealState->get()->bvec->data.begin()->index; // only one element should have prob 1.0
			map<string, string> unobsState = problem->getFactoredUnobservedStatesSymbols(yIndex);
			printTuple(unobsState);

			//initial belief Y state
			int mostProbY = iterBelief->get()->bvec->argmax(); 	//get the most probable Y state
			double prob = iterBelief->get()->bvec->operator()(mostProbY);	//get its probability
			streamOut.width(4); 
			streamOut << left << "ML Y" << ":";
			map<string, string> mostProbYState = problem->getFactoredUnobservedStatesSymbols(mostProbY);
			printTuple(mostProbYState);
			
			//action taken
			streamOut.width(4); streamOut << left << "A" << ":";
			map<string, string> actState = problem->getActionsSymbols(iterDecision->action);
			printTuple(actState);

			//reward for action & curretn state
			streamOut.width(4); streamOut << left << "R" << ":";
			streamOut << iterDecision->reward << endl;

			iterBelief++;
			iterRealState++;
			iterDecision++;
		}

	}
	else 
	{
		cout << "Failed to open file " << file << " !" << endl;
		return false;
	}

	return successful;
}

dataLogger::dataLogger(): streamOut("simOut.txt", ofstream::out | ofstream::app)
{

}


dataLogger::~dataLogger()
{
}
