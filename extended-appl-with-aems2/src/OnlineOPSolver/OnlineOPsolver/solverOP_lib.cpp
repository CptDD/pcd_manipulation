//
//	OnlineSolver/solverOP_lib.h
//
//	Copyright (c) 2015-2016 by Technical University of Cluj-Napoca.
//
//
//	Under GNU General Public License V2.
//
// this is a library file for ROS integration of the POMDP solver
#include "solverOP_lib.hpp"

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

void usage(const char* cmdName)
{
	cerr <<
		"Usage; " << cmdName << " POMDPModelFileName [--fast] [--precison targetPrecision] [--randomization]\n"
		"	[--timeout timeLimit] [--memory memoryLimit] [--output policyFileName] [--budget OPbudget] [--solverOption OPoption]\n"
		"	[--policy-interval timeInterval]\n"
		"    or " << cmdName << " --help (or -h)	Print this help\n"
		"    or " << cmdName << " --version		Print version information\n"
		"\n"
		"Solver options:\n"
		"  -f or --fast		Use fast (but very picky) alternate parser for .pomdp files.\n"
		"  -p or --precision targetPrecision\n"
		"			Set targetPrecision as the target precision in solution \n"
		"			quality; run ends when target precision is reached. The target\n"
		"			precision is 1e-3 by default.\n"
		"  --randomization	Turn on randomization for the sampling algorithm.\n"
		"			Randomization is off by default.\n"
		"  --timeout timeLimit	Use timeLimit as the timeout in seconds.  If running time\n"
		"			exceeds the specified value, the solver writes out a policy and\n"
		"			terminates. There is no time limit by default.\n"
		"  --memory memoryLimit	Use memoryLimit as the memory limit in MB. No memory limit\n"
		"			by default.  If memory usage exceeds the specified value,\n"
		"			ofsol writes out a policy and terminates. Set the value to be\n"
		"			less than physical memory to avoid swapping.\n"
		"  --trial-improvement-factor improvementConstant\n"
		"			Use improvementConstant as the trial improvement factor in the\n"
		"			sampling algorithm. At the default of 0.5, a trial terminates at\n"
		"			a belief when the gap between its upper and lower bound is 0.5 of\n"
		"			the current precision at the initial belief.\n"
		" --solverOption OPoption\n"
		"			POMDP online solver run potions as simulator or controller with or witout visualization\n"
		"			0 - simulate & visualise; 1 - control & visualise;\n"
		"			2 - simualte & not visualise; 3 control & not visualise\n"
		" --budget number\n"
		"			this is the budget for the OPMDP algorithm\n"
		" --UnObsInitStateFile fileName\n"
		"			the name of the file containing the code (in base of #swithe states) of the switch states like 10 = (OFF, ON) for 2 switches"
		"\n"
		"Policy output options:\n"
		"  -o or --output policyFileName\n"
		"			Use policyFileName as the name of policy output file. The\n"
		"			file name is 'out.policy' by default.\n"
		"  --policy-interval timeInterval\n"
		"			Use timeInterval as the time interval between two consecutive\n"
		"			write-out of policy files. If this is not specified, the solver\n"
		"			only writes out a policy file upon termination.\n"
		"\n"
		"Examples:\n"
		"  " << cmdName << " Hallway.pomdp\n"
		"  " << cmdName << " --timeout 100 --output hallway.policy Hallway.pomdp\n"
		"\n"
		;

	//		{"trial_improvement_factor",     1,NULL, 'j'}, // Use ARG as the trial improvement factor. The default is 0.5. So, for example, a trial terminates at a node when its upper and lower bound gap is less than 0.5 of the gap at the root.  


	/*	cerr <<
	"usage: " << cmdName << " OPTIONS <model>\n"
	"  -h or --help             Print this help\n"
	"  --version                Print version information\n"
	"\n"
	"Solver options:\n"
	"  -f or --fast             Use fast (but very picky) alternate POMDP parser\n"
	"  -p or --precision        Set target precision in solution quality; run ends when\n"
	"                           target is reached [default: 1e-3]\n"
	"  --randomization          Turn Randomization on for sampling\n"
	"\n"
	"Policy output options:\n"
	"  -o or --output           Specifies name of policy output file [default: 'out.policy']\n"
	"  --timeout                Specifies a timeout in seconds.  If running time exceeds\n"
	"                           the specified value, ofsol writes out a policy\n"
	"                           and terminates [default: no maximum]\n"
	"  --memory                 Specifies the maximum memory usage limit in mege bytes.  If memory usage exceeds\n"
	"                           the specified value, ofsol writes out a policy\n"
	"                           and terminates [default: no maximum]\n"
	"  --policy-interval        Specifies the time interval between two consecutive write-\n"
	"                           out of policy files\n"
	"\n"
	"Examples:\n"
	"  " << cmdName << " Hallway.pomdp\n"
	"  " << cmdName << " --timeout 100 --output hallway.policy Hallway.pomdp\n"
	"\n"
	;*/
	exit(-1);
}


int QMDPSolution(SharedPointer<MOMDP> problem, SolverParams* p)
{
	cout << "Generate QMDP Policy" << endl;
	double targetPrecision = MDP_RESIDUAL;
	// no need to invoke POMDP solver
	// solve MDP
	FullObsUBInitializer m;
	if (problem->XStates->size() != 1 && problem->hasPOMDPMatrices())
	{
		DEBUG_LOG(cout << "Calling FullObsUBInitialize::QMDPSolution_unfac()" << endl;);
		// un-factored 
		// only does this if convert fast is called to produce pomdp version of the matrices
		// need pomdp matrix
		m.QMDPSolution_unfac(problem, targetPrecision); // SYL030909 prevly: m.QValueIteration_unfac(problem, targetPrecision);
		int numActions = problem->actions->size();
		int numXstates = problem->XStates->size();
		int numYstates = problem->YStates->size();
		m.actionAlphaByState.resize(numActions);
		FOR(a, numActions)
		{
			m.actionAlphaByState[a].resize(numXstates);
			FOR(state_idx, numXstates)
			{
				m.actionAlphaByState[a][state_idx].resize(problem->getBeliefSize());
			}

		}

		FOR(a, numActions)
		{
			m.UnfacPostProcessing(m.actionAlphas[a], m.actionAlphaByState[a]);
		}
	}
	else
	{
		DEBUG_LOG(cout << "Calling FullObsUBInitialize::QMDPSolution()" << endl;);
		// factored
		m.QMDPSolution(problem, targetPrecision); // SYL030909 prevly: m.QValueIteration(problem, targetPrecision);
		FOR(a, problem->actions->size())
		{
			m.FacPostProcessing(m.actionAlphaByState[a]);
		}
	}

	AlphaPlanePoolSet alphaPlanePoolSet(NULL);
	alphaPlanePoolSet.setProblem(problem);
	alphaPlanePoolSet.setSolver(NULL);
	alphaPlanePoolSet.initialize();
	//addAlphaPlane(alphaPlane);

	FOR(a, problem->actions->size())
	{
		for (int stateidx = 0; stateidx < alphaPlanePoolSet.set.size(); stateidx++)
		{
			SharedPointer<AlphaPlane> plane(new AlphaPlane());
			copy(*plane->alpha, m.actionAlphaByState[a][stateidx]);
			plane->action = a;
			plane->sval = stateidx;

			alphaPlanePoolSet.set[stateidx]->addAlphaPlane(plane);
		}
	}
	string outFileName(p->outPolicyFileName);
	alphaPlanePoolSet.writeToFile(outFileName, p->problemName);
	return 0;
}

int FIBSolution(SharedPointer<MOMDP> problem, SolverParams* p)
{
	cout << "Generate FIB Policy" << endl;
	double targetPrecision = MDP_RESIDUAL;
	// no need to invoke POMDP solver

	FastInfUBInitializer f(problem);
	DEBUG_LOG(cout << "Calling FastInfUBInitializer::getFIBsolution()" << endl;);		f.getFIBsolution(targetPrecision);

	AlphaPlanePoolSet alphaPlanePoolSet(NULL);
	alphaPlanePoolSet.setProblem(problem);
	alphaPlanePoolSet.setSolver(NULL);
	alphaPlanePoolSet.initialize();
	//addAlphaPlane(alphaPlane);

	FOR(a, problem->actions->size())
	{
		for (int stateidx = 0; stateidx < alphaPlanePoolSet.set.size(); stateidx++)
		{
			SharedPointer<AlphaPlane> plane(new AlphaPlane());
			copy(*plane->alpha, f.actionAlphaByState[a][stateidx]);
			plane->action = a;
			plane->sval = stateidx;

			alphaPlanePoolSet.set[stateidx]->addAlphaPlane(plane);
		}
	}
	string outFileName(p->outPolicyFileName);
	alphaPlanePoolSet.writeToFile(outFileName, p->problemName);
	return 0;
}

int MDPSolution(SharedPointer<MOMDP> problem, SolverParams* p)
{
	cout << "Generate MDP Policy" << endl;
	double targetPrecision = MDP_RESIDUAL;
	// no need to invoke POMDP solver
	// solve MDP
	FullObsUBInitializer m;
	if (problem->XStates->size() != 1 && problem->hasPOMDPMatrices())
	{
		// un-factored 
		// only does this if convert fast is called to produce pomdp version of the matrices
		// need pomdp matrix
		m.alphaByState.resize(problem->XStates->size());
		DEBUG_LOG(cout << "Calling FullObsUBInitialize::valueIteration_unfac()" << endl;);
		m.valueIteration_unfac(problem, targetPrecision);
		m.UnfacPostProcessing(m.alpha, m.alphaByState);
	}
	else
	{
		// factored
		DEBUG_LOG(cout << "Calling FullObsUBInitialize::valueIteration()" << endl;);
		m.valueIteration(problem, targetPrecision);
		m.FacPostProcessing(m.alphaByState);
	}

	AlphaPlanePoolSet alphaPlanePoolSet(NULL);
	alphaPlanePoolSet.setProblem(problem);
	alphaPlanePoolSet.setSolver(NULL);
	alphaPlanePoolSet.initialize();
	//addAlphaPlane(alphaPlane);


	//do one step lookahead if problem is pure MDP
	if (problem->YStates->size() == 1)
	{
		for (int stateidx = 0; stateidx < alphaPlanePoolSet.set.size(); stateidx++)
		{
			SharedPointer<AlphaPlane> plane(new AlphaPlane());
			int maxAction = 0;
			double maxActionLB = -DBL_MAX;

			//search for the best action for this state
			SharedPointer<BeliefWithState> b = SharedPointer<BeliefWithState>(new BeliefWithState);
			b->bvec = new SparseVector(); b->bvec->resize(1);
			b->bvec->push_back(0, 1.0); b->sval = stateidx;
			//initialise the MDP belief to current state
			obsState_prob_vector spv;  // outcome probability for values of observed state
			for (Actions::iterator aIter = problem->actions->begin(); aIter != problem->actions->end(); aIter++)
			{
				int a = aIter.index();

				double sum = 0.0;
				double immediateReward = problem->rewards->getReward(*b, a);
				problem->getObsStateProbVector(spv, *b, a);

				FOR(Xn, spv.size())
				{
					double sprob = spv(Xn);
					if (sprob > OBS_IS_ZERO_EPS)
					{
						double childLB = m.alphaByState[Xn](0);
						sum += childLB * sprob;
					}
				}
				sum *= problem->getDiscount();
				sum += immediateReward;

				if (sum > maxActionLB)
				{
					maxActionLB = sum;
					maxAction = a;
				}
				assert(maxActionLB != -DBL_MAX);
			}

			copy(*plane->alpha, m.alphaByState[stateidx]);
			plane->action = maxAction;
			plane->sval = stateidx;

			alphaPlanePoolSet.set[stateidx]->addAlphaPlane(plane);
		}
	}
	else {
		for (int stateidx = 0; stateidx < alphaPlanePoolSet.set.size(); stateidx++)
		{
			SharedPointer<AlphaPlane> plane(new AlphaPlane());
			copy(*plane->alpha, m.alphaByState[stateidx]);
			plane->action = -1;
			plane->sval = stateidx;

			alphaPlanePoolSet.set[stateidx]->addAlphaPlane(plane);
		}
	}

	string outFileName(p->outPolicyFileName);
	alphaPlanePoolSet.writeToFile(outFileName, p->problemName);
	return 0;
}

int randomPDF(SharedPointer<DenseVector> pdf)
{
	int res = -1;
	double randomNr = (double)(rand() % 101)/100.0;
	double begin = 0.0, end = 0.0;

	FOR(i, pdf->size())
	{
		end = begin + pdf->data[i];

		if (begin <= randomNr && randomNr <= end)
		{
			res = i;
			return i;
		}
		begin = begin + pdf->data[i];
	}

	//assert(res != -1, "wrong pdf was given");
	return -1;
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
#ifdef OPENCV
Point getCurrentPoint(SharedPointer<MOMDP> problem)
{

	Point ret;

	int pozitionID = problem->initialBeliefStval->sval;
	map<string, string> pozitionName;
	pozitionName = problem->getFactoredObservedStatesSymbols(pozitionID);
	//int coordX = -1, coordY = -1;
	string sX = "", sY = "";
	sY = pozitionName.begin()->second;

	stringstream sstmp(sY);
	
	//std::string::size_type sz;   // alias of size_t
	string stmp;
	getline(sstmp, stmp, '_');
	

	int coordY=atoi( stmp.c_str() );

	//stmp >> coordY;
	//coordY = stoi(sY, &sz);

//	sX = sY.substr(sz + 1);	

	getline(sstmp,stmp);	
	int coordX =atoi( stmp.c_str() );
	
//	coordX = stoi(sX);

	ret.x = coordX * 100 - 50;
	ret.y = coordY * 100 - 50;

	return ret;
}
#endif
// by Elod Pall read in form file the int representation of sw states

double getReward(SharedPointer<MOMDP> problem, const BeliefWithState& belst, int action)
{
	//const SparseMatrix rewMat = problem->getRewardMatrix(belst.sval);
	const SharedPointer<SparseMatrix>  rewMat = problem->rewards->getMatrix(belst.sval);
	return inner_prod_column(*rewMat, action, *belst.bvec);
}
#ifdef OPENCV
void visualiseMovement(SharedPointer<MOMDP> problem, Mat image, Point &prevPoint, int a)
{
	Point currPoint = getCurrentPoint(problem);
	arrowedLine(image, prevPoint, currPoint, Scalar(1, 200 - 1, 1), 4, 8, 0, 0.1);
	imshow("Display Image", image);
	waitKey(1);
	prevPoint = currPoint;
	switch (a)
	{
	case 4:
		circle(image, currPoint, 15, Scalar(200 - 1, 220, 222), 15, 8, 0);
		break;
	case 5:
		putText(image, "X", currPoint - Point(5, -10),
			FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(1, 200 - 1, 1), 2, CV_AA);
		break;
	default:
		break;
	}
	imshow("Display Image", image);
	waitKey(30);
}
#endif

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

///===================================================================///
///===================================================================///
///===================================================================///
POMDPplanner::POMDPplanner( char* model, int budget, bool simulate, bool visualise, int simLength, int simTime,  char* outputFile,  char* UnObsInitStateFile)
{
	p = &GlobalResource::getInstance()->solverParams;

	this->simulation = simulate;
	this->visualization =visualise;

	p->cmdName = model;  		// path and file
	p->OPbudget = budget;		// expansion budget
	p->simLen = simLength;		// number of simulated steps
	p->simNum = simTime;		// time duration [ms] for building the tree
	p->problemName = model;
	p->outPolicyFileName = string(outputFile);
	p->initialUnObsState_file = string(UnObsInitStateFile);
	GlobalResource::getInstance()->init();
	string baseName = GlobalResource::getInstance()->parseBaseNameWithoutPath(p->problemName);
	GlobalResource::getInstance()->setBaseName(baseName);
	cout << "\nLoading the model: " << baseName << " ...\n" ;
	GlobalResource::getInstance()->PBSolverPrePOMDPLoad();
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
	dataLog.setup(problem, p->outPolicyFileName);
	lbBackup = new BackupAlphaPlaneMOMDP();
	ubBackup = new BackupBeliefValuePairMOMDP();
	opSolver = new OPMDP(problem, p);
	lbBackup->problem = problem;
	opSolver->lowerBoundBackup = lbBackup;

	((BackupAlphaPlaneMOMDP*)(opSolver->lowerBoundBackup))->solver = opSolver;

	ubBackup->problem = problem;
	opSolver->upperBoundBackup = ubBackup;
	solver = opSolver;
	currentAction = -1;
	newUnobsState = 0;
	currObservation = 0;
}
POMDPplanner::~POMDPplanner(void)
{ }

int POMDPplanner::returnInitialUnObsSt(const char * file)
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

void POMDPplanner::simulateAll()
{

#ifdef OPENCV
	// intit visualization
	Point prevPoint, currPoint;
	Mat image, originalImg;

	if (visualise)
	{

		prevPoint = getCurrentPoint(problem);

		image = imread("map.png", 1);
		originalImg = image.clone();
		if (!image.data)
		{
			printf("No image data \n");
			visualise = false;
		}
		else
		{
			//cout << "Image of map loaded ...";
			namedWindow("Display Image", CV_WINDOW_AUTOSIZE);
			imshow("Display Image", image);
			cvWaitKey(1);
		}
	}
#endif
	// run simulation
	int simulationLength = p->simLen;

	if (p->initialUnObsState_file != "") {
		p->initialUnObsState_val = returnInitialUnObsSt(
				p->initialUnObsState_file.c_str());
	} else {
		p->initialUnObsState_val = 0; // meaning that no initial state is given for unobservable St var-s so all swithces are on
	}

	SharedPointer<BeliefWithState> actStateCompl(new BeliefWithState());
	SharedPointer<BeliefWithState> actNewStateCompl(new BeliefWithState());

	int belSize = problem->initialBeliefStval->bvec->size();
	int currentUnobsState = p->initialUnObsState_val;

	actStateCompl->sval = problem->initialBeliefStval->sval;
	actStateCompl->bvec->resize(belSize);
	actStateCompl->bvec->push_back(currentUnobsState, 1.0);

	double reward = 0.0;
	int currentAction = -1, newUnobsState = 0, currObservation = 0;
	uint64 Tbegin = 0.0, Tend = 0.0;
//	Tbegin = GetTimeMs64_lib();
	cout << endl << "=   In simulation...   =" << endl;
	for (int simIndex = 0; simIndex < simulationLength; simIndex++) {
		// solve problem and get besta action
		currentAction = solver->getOptimalAction(problem, currentAction,
				currObservation, actNewStateCompl->sval);

		// simulate action and overwrite Initial state in problem
		//simualteCurrentAction(problem, currentAction, true); // maybe I should rewrite it

		double currReward = getReward(problem, *actStateCompl, currentAction);
#ifdef OPENCV
		if (visualise)
		{
			visualiseMovement(problem, image, prevPoint, currentAction);
		}
#endif
		// actualActionUpdObs is belief of the fully observered state
		belief_vector actualActionUpdUnobs(belSize), actualActionUpdObs(
				problem->XStates->size());
		performActionObs(problem, actualActionUpdObs, currentAction,
				*actStateCompl);
		actNewStateCompl->sval = (unsigned int) chooseFromDistribution(
				actualActionUpdObs, ((double) rand() / RAND_MAX));

		// now update actualActionUpdUnobs, which is the belief of unobserved states,
		// based on prev belif and curr observed state
		performActionUnobs(problem, actualActionUpdUnobs, currentAction,
				*actStateCompl, actNewStateCompl->sval);

		// the actual next state for the unobserved variable
		newUnobsState = chooseFromDistribution(actualActionUpdUnobs,
				((double) rand() / RAND_MAX));

		actNewStateCompl->bvec->resize(belSize);
		actNewStateCompl->bvec->push_back(newUnobsState, 1.0);

		belief_vector obsPoss;
		getPossibleObservations(problem, obsPoss, currentAction,
				*actNewStateCompl);

		currObservation = chooseFromDistribution(obsPoss,
				((double) rand() / RAND_MAX));

		dataLog.addData(problem->initialBeliefStval, actStateCompl,
				currentAction, currObservation, actNewStateCompl->sval,
				currReward);

		//actual states
		currentUnobsState = newUnobsState; //Y state, hidden
		actStateCompl->sval = actNewStateCompl->sval;
		copy(*actStateCompl->bvec, *actNewStateCompl->bvec);

		resetProblemInit(problem, problem->initialBeliefStval, currentAction,
				currObservation, actNewStateCompl->sval);
		//cout << simIndex <<"  a: " << currentAction << "  r: " << currReward << endl;

//		Tend = GetTimeMs64_lib();
		cout << "========= Execution time for OPMDP solution: " << Tend - Tbegin
				<< endl;
		cout << "========= Reward for discount " << problem->discount
				<< " at the end of simulation: "
				<< dataLog.calculateReward(problem->discount) << endl;

		dataLog.printToFile();

#ifdef REWARD_TESTING
		ofstream outRfile;
		outRfile.open("simulationR.txt", ofstream::out | ofstream::app);
		if (outRfile.is_open()) {
			if (p->simNum <= 0)
				// outRfile << "executaion_time | budget | #simulationSteps | reward"
				outRfile << Tend - Tbegin << " " << p->OPbudget << " "
						<< simulationLength << " "
						<< dataLog.calculateReward(problem->discount) << endl;
			else
				outRfile << Tend - Tbegin << " " << p->simNum << " "
						<< simulationLength << " "
						<< dataLog.calculateReward(problem->discount) << endl;
		}
		outRfile.close();

		cout << "results written to simulationR.txt " << endl;
#endif

	}

	//int curretnAction = solver->getOptimalAction(problem);
#ifdef OPENCV

	waitKey();
#endif

}


int POMDPplanner::nextIteration(int obseravtion, int position)
{
	if (simulation)
	{
		std::cout << "simulation started...\n";
		simulateAll();
		return 0;
	}
	else
	{
///================== REAL CASE SENARIO =============================///
		if (p->initialUnObsState_file != "")
		{
			p->initialUnObsState_val = returnInitialUnObsSt(p->initialUnObsState_file.c_str());
		}
		else
		{
			p->initialUnObsState_val = 0; // meaning that no initial state is given for unobservable St var-s so all swithces are on
		}

		std::cout << "one step calculation started..\n";
		currentAction = solver->getOptimalAction(problem,currentAction, obseravtion, position);

//		====> unknown if this initial state must be updated and where in the algorithm will be used
//		resetProblemInit(problem, problem->initialBeliefStval, currentAction, currObservation, actNewStateCompl->sval);
	}
	std::cout << "planner terminated, A = " << currentAction << std::endl ;
	return currentAction;
}


