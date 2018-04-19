#include "treeHandler.h"

#ifdef _WIN32
#include <windows.h> // Windws insludes
//#include "stdafx.h" //  Windows stuff
#endif

#include <stdio.h>
#include <stdlib.h>
#include <cfloat>

#ifdef _MSC_VER
#include "getopt.h"
#define NOMINMAX 


#else
#include <getopt.h>
#include <sys/time.h>
#endif

#include <signal.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <assert.h>
#include <cstring>
#include <cstdlib>


#include "CPTimer.h"

#include "GlobalResource.h"
//#include "ActionSelector.h"
//#include "PolicyFollower.h"

#include "solverUtils.h"
#include "Parser.h"
#include "POMDP.h"
#include "ParserSelector.h"
#include "MOMDP.h"
#include "SARSOP.h"

#include "OPMDP.h"
#include "BackupAlphaPlaneMOMDP.h"
#include "BackupBeliefValuePairMOMDP.h"
//#include "dataLogger.h"

//#include "FSVI.h"
//#include "GES.h"
#include "FullObsUBInitializer.h"
#include "FastInfUBInitializer.h"



// visualisation libraries		PAEL
//#include "opencv\highgui.h"
//#include "opencv\cv.h"
/* // Windws insludes
#include "opencv2\core.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\imgproc.hpp"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"

#include "opencv2/highgui/highgui_c.h"
v
*/
#define TREETEST


#undef DEBUG_TRACE_ON 
#define REWARD_TESTING

#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#include <ctime>
#endif


using namespace std;
using namespace momdp;

/* Remove if already defined */
typedef long long int64; typedef unsigned long long uint64;

/* Returns the amount of milliseconds elapsed since the UNIX epoch. Works on both
* windows and linux. */

uint64 GetTimeMs64_lib()
{
#ifdef _WIN32
	/* Windows */
	FILETIME ft;
	LARGE_INTEGER li;

	/* Get the amount of 100 nano seconds intervals elapsed since January 1, 1601 (UTC) and copy it
	* to a LARGE_INTEGER structure. */
	GetSystemTimeAsFileTime(&ft);
	li.LowPart = ft.dwLowDateTime;
	li.HighPart = ft.dwHighDateTime;

	uint64 ret = li.QuadPart;
	ret -= 116444736000000000LL; /* Convert from file time to UNIX epoch time. */
	ret /= 10000; /* From 100 nano seconds (10^-7) to 1 millisecond (10^-3) intervals */

	return ret;
#else
	/* Linux */
	struct timeval tv;

	gettimeofday(&tv, NULL);

	uint64 ret = tv.tv_usec;
	/* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
	ret /= 1000;

	/* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
	ret += (tv.tv_sec * 1000);

	return ret;
#endif
}

#ifdef __cplusplus
extern "C"
#endif
{
	extern unsigned long GlobalMemLimit;
}

struct OutputParams_lib {
	double timeoutSeconds;
	double interval;
	OutputParams_lib(void);
};

OutputParams_lib::OutputParams_lib(void) {
	timeoutSeconds = -1;
	interval = -1;
}


#ifdef _MSC_VER
BOOL CtrlHandler(DWORD fdwCtrlType)
{
	switch (fdwCtrlType)
	{
		// Handle the interrupt signal. 
	case CTRL_C_EVENT:
	case CTRL_CLOSE_EVENT:
	case CTRL_BREAK_EVENT:
	case CTRL_SHUTDOWN_EVENT:
	case CTRL_LOGOFF_EVENT:
		if (GlobalResource::getInstance()->solving)
		{
			GlobalResource::getInstance()->userTerminatedG = true;
		}
		else
		{
			exit(1);
		}
		printf("*** Received SIGINT. User pressed control-C. ***\n");
		printf("\nTerminating ...\n");
		fflush(stdout);
		GlobalResource::getInstance()->userTerminatedG = true;
		return(TRUE);

	default:
		return FALSE;
	}
}

void registerCtrlHanler()
{
	if (SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE))
	{
		// Success
	}
	else
	{
		// Failed to register... but continue anyway
		printf("\nERROR: Could not set control handler");
	}
}

#else

void sigIntHandler(int sig) {

	if (GlobalResource::getInstance()->solving)
	{
		GlobalResource::getInstance()->userTerminatedG = true;
	}
	else
	{
		exit(1);
	}


	printf("*** Received SIGINT. User pressed control-C. ***\n");
	printf("\nTerminating ...\n");
	fflush(stdout);
}

void setSignalHandler(int sig, void(*handler)(int))
{
	struct sigaction act;
	memset(&act, 0, sizeof(act));
	act.sa_handler = handler;
	act.sa_flags = SA_RESTART;
	if (-1 == sigaction(sig, &act, NULL)) {
		cerr << "ERROR: unable to set handler for signal "
			<< sig << endl;
		exit(EXIT_FAILURE);
	}


}
#endif


// b represents a discrete probability distribution Pr(outcome = i) = b(i).
// Chooses an outcome according to the distribution.
/*
inline int chooseFromDistribution(const DenseVector& b)
{
	double r = unit_rand();
	FOR(i, b.size())
	{
		r -= b(i);
		if (r <= 0)
		{
			return i;
		}
	}
	return b.data.size() - 1;
}

// simultes curent action and generates all outcomes based on pdf functions, overwrites initial state in problem
void simualteCurrentAction(SharedPointer<MOMDP>problem, int currentAction, bool random)
{
	//double reward = 0.0;

	// set up successors for this fringe node (possibly creating new fringe nodes)
	obs_prob_vector  opv; // outcome_prob_vector opv;
	//state_vector *sp = new state_vector;
	SharedPointer<SparseVector> jspv(new SparseVector());

	//vector<SharedPointer<SparseVector> > newInitialBeliefYByX;
	SharedPointer<SparseVector> newInitialBeliefY(new SparseVector());	//POMDP vector	
	SharedPointer<BeliefWithState>  newInitialBeliefStval(new BeliefWithState());	//MDP + POMDP structure
	SharedPointer<DenseVector> newInitialBeliefX(new DenseVector(problem->initialBeliefX->size()));	// MDP vector 

	obsState_prob_vector spv;  // outcome probability for values of observed state
	belief_vector currStateVector;	// MDP state vector
	SharedPointer<BeliefWithState> currSvalBvec(new BeliefWithState()); // state_vector of MDP and POMDP states = sp;	
	int newSval = 0;			// next MDP state valeu
	int currSval = 0;			// current MDP state valeu


	copy(currStateVector, *(problem->initialBeliefX));
	float maxStvalprob = 0.0;

	if (random)
	{
		currSval = randomPDF(problem->initialBeliefX);
	}
	else
	{
		FOR(i, currStateVector.size())
		{
			if (currStateVector(i) > maxStvalprob)
			{
				currSval = i;
				maxStvalprob = currStateVector(i);
			}
		}
	}


	copy(*currSvalBvec->bvec, *problem->getInitialBeliefY(currSval));
	currSvalBvec->bvec->finalize();
	currSvalBvec->sval = currSval;

	problem->getObsStateProbVector(spv, *currSvalBvec, currentAction); // P(Xn|cn.s,a)

	double tmpMaxXn = 0;
	int tmpMaxXnID = 0;
	cout << endl << spv.data.size() << endl;
	FOR(i, spv.data.size())
	{
		newInitialBeliefX->data[spv.data[i].index] = spv.data[i].value;
		if (tmpMaxXn < spv.data[i].value)
		{
			tmpMaxXn = spv.data[i].value;
			tmpMaxXnID = spv.data[i].index;
		}
	}

	if (random)
	{
		newSval = randomPDF(newInitialBeliefX);
	}
	else
	{
		newSval = tmpMaxXnID;
	}
	// BUG jspv not calculated OK!
	problem->getJointUnobsStateProbVector(*jspv, currSvalBvec, currentAction, newSval);
	problem->getObsProbVectorFast(opv, currentAction, newSval, *jspv); // only the joint prob is useful for later but we calculate the observation prob P(o|Xn,cn.s,a)

	int newObservationPoint = 0;

	double tmpMaxO = 0;
	int tmpMaxOID = 0;
	SharedPointer<DenseVector> newObservationPDF(new DenseVector(opv.size()));	// MDP vector 
	FOR(i, opv.data.size())
	{
		newObservationPDF->data[opv.data[i].index] = opv.data[i].value;
		if (tmpMaxO < opv.data[i].value)
		{
			tmpMaxO = opv.data[i].value;
			tmpMaxOID = opv.data[i].index;
		}
	}

	if (random)
	{
		newObservationPoint = randomPDF(newObservationPDF);
	}
	else
	{
		newObservationPoint = tmpMaxOID;
	}

	newInitialBeliefStval = (problem->beliefTransition->nextBelief2(currSvalBvec, currentAction,
		newObservationPoint, newSval, jspv));
	newInitialBeliefStval->bvec->finalize();
	newInitialBeliefStval->sval = newSval;

	newInitialBeliefY = newInitialBeliefStval->bvec;


	copy(*problem->initialBeliefY, *newInitialBeliefY);
	copy(*problem->initialBeliefStval->bvec, *newInitialBeliefStval->bvec);
	problem->initialBeliefStval->bvec->finalize();
	problem->initialBeliefStval->sval = newSval;

	//return reward;
}

void resetProblemInit(SharedPointer<MOMDP> problem, SharedPointer<BeliefWithState> currentBel, int action, int observation, int Xn)
{
	SharedPointer<BeliefWithState> nextBelSt;

	nextBelSt = problem->beliefTransition->nextBelief(currentBel, action, observation, Xn);

	SharedPointer<DenseVector> newInitialBeliefX(new DenseVector(problem->initialBeliefX->size()));	// MDP vector 	
	newInitialBeliefX->data[Xn] = 1.0;

	copy(*problem->initialBeliefX, *newInitialBeliefX);
	copy(*problem->initialBeliefY, *nextBelSt->bvec);
	copy(*problem->initialBeliefStval->bvec, *nextBelSt->bvec);
	problem->initialBeliefStval->bvec->finalize();
	problem->initialBeliefStval->sval = nextBelSt->sval;
}

// by Elod Pall read in form file the int representation of sw states
int returnInitialUnObsSt(const char * file)
{
	int p = 0, nr = 0;
	unsigned bit = 0, base = 2;

	fstream initFile;
	initFile.open(file, std::ios_base::in);
	if (initFile.is_open())
	{
		initFile >> nr;
		for (int i = 0; i<nr; i++)
		{
			initFile >> bit;
			p = p + bit*pow(base, nr - (i + 1));
		}
		cout << "\nInitial UnObs state loaded ... ";
		cout << " input file	: " << file << " value = " << p << endl;
	}
	else
	{
		return -1;
		cout << "\n The file " << file << " can not be opend! \n";
	}
	return p;
}

void performActionObs(SharedPointer<MOMDP> problem, belief_vector& outBelObs, int action, const BeliefWithState& belSt)
{
	// DEBUG_SIMSPEED_270409 skip calculating outprobs for x when there is only one possible x value
	if (problem->XStates->size() == 1)
	{
		// clear out the entries
		outBelObs.resize(1);
		outBelObs.push_back(0, 1.0);
	}
	else
	{
		//problem->getTransitionMatrixX(action, belSt.sval);
		const SharedPointer<SparseMatrix>  transMatX = problem->XTrans->getMatrix(action, belSt.sval);
		mult(outBelObs, *belSt.bvec, *transMatX);
	}
}

void performActionUnobs(SharedPointer<MOMDP> problem, belief_vector& outBelUnobs, int action, const BeliefWithState& belSt, int currObsState)
{
	const SharedPointer<SparseMatrix>  transMatY = problem->YTrans->getMatrix(action, belSt.sval, currObsState);
	mult(outBelUnobs, *belSt.bvec, *transMatY);
}

void getPossibleObservations(SharedPointer<MOMDP> problem, belief_vector& possObs, int action, const BeliefWithState& belSt)
{
	//const SparseMatrix obsMat = problem->getObservationMatrix(action, belSt.sval);
	const SharedPointer<SparseMatrix>  obsMat = problem->obsProb->getMatrix(action, belSt.sval);
	mult(possObs, *belSt.bvec, *obsMat);
}
*/
//===========================================================
//=================== testing ===============================
//===========================================================

int freeNodeNR = 0;










int main(int argc, char **argv)
{
	//try
	{
		/* initialize random seed: */
		srand(time(NULL));

		SolverParams* p = &GlobalResource::getInstance()->solverParams;



		bool parseCorrect = SolverParams::parseCommandLineOption(argc, argv, *p);
		if (!parseCorrect)
		{
			exit(EXIT_FAILURE);
		}


		OutputParams_lib op;
		if (GlobalResource::getInstance()->benchmarkMode)
		{
			if (GlobalResource::getInstance()->simNum == 0 || GlobalResource::getInstance()->simLen == 0)
			{
				cout << "Benchmark Length and/or Number not set, please set them using option --simLen and --simNum" << endl;
				exit(-1);
			}
		}


		GlobalResource::getInstance()->init();
		string baseName = GlobalResource::getInstance()->parseBaseNameWithoutPath(p->problemName);
		GlobalResource::getInstance()->setBaseName(baseName);

		//*************************
		//TODO: parse the problem
		//	long int clk_tck = sysconf(_SC_CLK_TCK);
		//	struct tms now1, now2;
		//	float utime, stime;

#ifdef _MSC_VER
		registerCtrlHanler();
#else
		setSignalHandler(SIGINT, &sigIntHandler);
#endif

		printf("\nLoading the model ...\n  ");

		//Parser* parser = new Parser();  

		GlobalResource::getInstance()->PBSolverPrePOMDPLoad();
		SharedPointer<MOMDP> problem(NULL);
		if (p->hardcodedProblem.length() == 0)
		{
			problem = ParserSelector::loadProblem(p->problemName, *p);
		}
		else
		{
			cout << "Unknown hard coded problem type : " << p->hardcodedProblem << endl;
			exit(0);
		}

		double pomdpLoadTime = GlobalResource::getInstance()->PBSolverPostPOMDPLoad();
		printf("  loading time : %.2fs \n", pomdpLoadTime);
		GlobalResource::getInstance()->problem = problem;

	
		treeHandler * tester = new treeHandler(problem, p);

		tester->runMemFreeTest(3, false);



	}


	return 0;
}

