//
//	Algorithms/OPMDP/OPMDP.cpp
//
//	Copyright (c) 2008-2010 by National University of Singapore.
//	Modifications Copyright (c) 2015-2016 by Technical University of Cluj-Napoca.
//
//
//	Under GNU General Public License V2.
//

#include <cerrno>
#include <cstring>
#include <iomanip>

#include "FHHOP.h"
#include "SARSOPPrune.h"
#include "MOMDP.h"
#include "BeliefValuePairPoolSet.h"
#include "AlphaPlanePoolSet.h"
#include "BeliefTreeNode.h"
#include "CPTimer.h"
#include "BlindLBInitializer.h"
#include "FastInfUBInitializer.h"
#include "BackupBeliefValuePairMOMDP.h"
#include "BackupAlphaPlaneMOMDP.h"

using std::swap;

#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#include <ctime>
#endif

/* Remove if already defined */
typedef long long int64; typedef unsigned long long uint64;

/* Returns the amount of milliseconds elapsed since the UNIX epoch. Works on both
 * windows and linux. */

uint64 GetTimeMs64_OP()
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

// get oprimal action
void FHHOP::solve(SharedPointer<MOMDP> problem)
{
}

FHHOPBeliefTreeNode * FHHOP::freeSubTree(FHHOPBeliefTreeNode *cn, int A, int O, int X, bool firstLeve)
{
	if (cn->isFringe())
	{
		cn->parent = NULL;
		cn->optimalLeafNode_AEMS2 = NULL;
		cn->optimalLeafNode_FHHOP = NULL;
		delete cn;
	}
	else
	{
		FOR(Aval, cn->Q.size())
		{
			FHHOPBeliefTreeQEntry * Qa = &cn->Q[Aval];

			FOR(Xni, Qa->getNumStateOutcomes())
			{
				FHHOPBeliefTreeObsState * Xn = Qa->stateOutcomes[Xni];

				if (Xn)
				{
					FOR(Oval, Xn->getNumOutcomes())
					{
						FHHOPBeliefTreeEdge *Xo = Xn->outcomes[Oval];
						if (Xo)
						{
							//in order to use in real app, we should exclude the full obs state X
							//only one Xo will be initialized in the list and should be found at the first instance
							//need to test it !!!! && X == Xni
							if (A == Aval && O == Oval && firstLeve)
							{
								Xo->nextState->parent = NULL;
								return Xo->nextState;
							}
							else
							{
								freeSubTree(Xo->nextState, -1, -1, -1, false);
								Xo->nextState = NULL;
							}

						}
					}
				}
			}
		}
		delete cn;
	}
}

int FHHOP::getOptimalAction(SharedPointer<MOMDP> problem, int action, int observation, int Xn)
{
	// init root or free subtree
	if (0 <= action &&  this->solutionRootNode)
	{
		this->solutionRootNode = freeSubTree(this->solutionRootNode, action, observation, Xn, true);
		this->depthOffset++;
	}
	else
	{
		this->solutionRootNode = initOPRoot();
		cout << "init root done" << endl;
	}

	//re init node selection parameters
	this->I_AEMS2 = 0;
	this->I_FHHOP = 0;
	this->N_AEMS2 = 0;
	this->N_FHHOP = 0;

	// buld optimal tree
	generateTree(problem, action, observation, Xn);

	//search optimal action
	int maxAction = -1;
	double maxLb = -99e+20;

	FOR(a, this->solutionRootNode->getNodeNumActions())
	{
		FHHOPBeliefTreeQEntry & Qa = this->solutionRootNode->Q[a];
		if (maxLb < Qa.lbVal)
		{
			maxLb = Qa.lbVal;
			maxAction = a;
		}
	}
	cout << "nr selections AEMS2: " << N_AEMS2 << " FHHOPP: " << N_FHHOP << endl;
	return maxAction;
}



// propagate lower and upper bound, aslo select besta action	PAEL
// we are not going to use the returned max action ID
int FHHOP::backupUb_Lb(FHHOPBeliefTreeNode* cn)
{
	if (cn->isFringe())
		return -1;

	double reward = 0.0, gamma = 0.0, cn_la = 0.0, cn_ua = 0.0;
	int maxAction = -1;
	double maxUB = -99e+20, maxLB = -99e+20;
	gamma = this->problem->discount;

	FOR(a, cn->getNodeNumActions())
	{
		REAL_VALUE vUa = 0.0, vLa = 0.0;

		FHHOPBeliefTreeQEntry & Qa = cn->Q[a];

		reward = Qa.immediateReward;
		cn_la = 0.0;
		cn_ua = 0.0;

		FOR(xn, Qa.getNumStateOutcomes())
		{
			FHHOPBeliefTreeObsState* Qax = Qa.stateOutcomes[xn];
			if (Qax)
				FOR(o, Qax->getNumOutcomes())
			{
				if (Qax->outcomes[o])
				{
					FHHOPBeliefTreeEdge *e = Qax->outcomes[o];

					cn_la = cn_la + e->obsProb * (reward + gamma * e->nextState->LB);
					cn_ua = cn_ua + e->obsProb * (reward + gamma * e->nextState->UB);
				}

			}
		}
		Qa.lbVal = cn_la;
		Qa.ubVal = cn_ua;

		if (cn_ua > maxUB)
		{
			maxUB = cn_ua;
			maxAction = a;
		}

		if (cn_la > maxLB)
		{
			maxLB = cn_la;
		}

	}
	cn->deltaLB = abs(cn->LB - maxLB);
	cn->deltaUB = abs(cn->UB - maxUB);

	cn->LB = maxLB;
	cn->UB = maxUB;

	return maxAction;
}

// propagate max contribution and optimal leaf node		PAEL
// we look through all the actions not only on the optimal one!
void FHHOP::backupContributution_AEMS2(FHHOPBeliefTreeNode* cn, int maxAction)
{
	FHHOPBeliefTreeQEntry & Qa = cn->Q[maxAction];

	double maxContribution_AEMS2 = -99e+20;
	FHHOPBeliefTreeNode* maxContributionNode_AEMS2 = NULL;

	FOR(xn, Qa.getNumStateOutcomes())
	{
		FHHOPBeliefTreeObsState* Qax = Qa.stateOutcomes[xn];
		if (Qax)
			FOR(o, Qax->getNumOutcomes())
			{
				FHHOPBeliefTreeEdge *e = Qax->outcomes[o];
				if (e) {

					// AMES2 contribution backup only for optimal action
					if (e->nextState->contribution_AEMS2 > maxContribution_AEMS2) {
						maxContribution_AEMS2 =
								e->nextState->contribution_AEMS2;
						maxContributionNode_AEMS2 = e->nextState;
					}
				}
			}
	}

	// save the leafe node that  should be expanded at next step PAEL
	cn->optimalLeafNode_AEMS2 = maxContributionNode_AEMS2->optimalLeafNode_AEMS2;
	cn->contribution_AEMS2 = maxContribution_AEMS2;
}

// Backup the expanded node for AEMS2		// PAEL
void FHHOP::backupOPNode(FHHOPBeliefTreeNode* cn)
{
	do
	{
		// maxAction is used only in AEMS2
		int maxAction = backupUb_Lb(cn);
		//ssert(maxAction != -1 " the node requested for backup is a finge node!");

		// backup contribution & optimal leaf selection for AEMS2
		backupContributution_AEMS2(cn, maxAction);
		cn = cn->parent;
	} while (cn);
}

// check if the node is fringe and then expand it.		PAEL
void FHHOP::expandPrepare(FHHOPBeliefTreeNode* cn)
{
	if (cn->isFringe())
	{
		expandOP(*cn);
		//cout << "OK";
	}
	else
	{
		cout << "\n ERROR = the given node for expantion is not fringe \n";
		//exit -1;
	}
}

void onGetOPNode(FHHOP *solver, FHHOPBeliefTreeNode* node, SharedPointer<BeliefWithState>& belief)
{

	//OPMDP *opSolver = (OPMDP *)solver;
	//int stateidx = belief->sval;
	//			int row = node->cacheIndex.row;
	//int timeStamp = solver->numBackups;

	// SARSOP Bin Manager related
	//			sarsopSolver->binManagerSet->binManagerSet[stateidx]->binManagerDataTable.set(row).binned = false;

	// TODO: fix this bug, UB_ACTION is set by backup, but if a node is allocated and sampled before backup, UB_ACTION is not defined
	//			sarsopSolver->upperBoundSet->set[stateidx]->dataTable->set(row).UB_ACTION = 0;


	//list<SharedPointer<AlphaPlane> >* alphas = new list<SharedPointer<AlphaPlane> >();
	//			sarsopSolver->lowerBoundSet->set[stateidx]->dataTable->set(row).ALPHA_PLANES = alphas;

	// TODO:: fix it
	//SharedPointer<AlphaPlane>alpha = solver->lowerBoundSet->getValueAlpha(belief);
	//REAL_VALUE lbVal = sarsopSolver->lowerBoundSet->getValue(belief);
	REAL_VALUE lbVal = inner_prod(*solver->lowerBoundSet->getValueAlpha(belief)->alpha, *belief->bvec);

	//REAL_VALUE lbVal = bounds->getLowerBoundValue(b_s, &alpha);
	//OPMDPAlphaPlaneTuple *dataAttachedToAlpha = (OPMDPAlphaPlaneTuple *)(alpha->solverData);

	REAL_VALUE ubVal = solver->upperBoundSet->getValue(belief);

	node->LB =  lbVal;
	node->UB =  ubVal;


	//if (timeStamp != -1)
	//{
	//	//assert(solver->beliefCacheSet[stateidx]->getRow( row)->isFringe );
	//	DEBUG_TRACE("getNode timeStamp!=-1");
	//	if (true)//(!hasMaxMetaAt(alpha, node->cacheIndex.row))	<- original // assume that the alpha is from the correct boundsSet[]  //PAEL
	//	{
	//		DEBUG_TRACE("!hasMaxMetaAt");
	//		AlphaPlaneMaxMeta* newMax = new AlphaPlaneMaxMeta();
	//		//					newMax->cacheIndex = node->cacheIndex.row;
	//		newMax->lastLB = lbVal;
	//		newMax->timestamp = GlobalResource::getInstance()->getTimeStamp();
	//		dataAttachedToAlpha->maxMeta.push_back(newMax);
	//	}
	//}

	//opSolver = NULL;
}

// create a new fringe node with: s_Val b_Vec UB LB optimal_Leaf		PAEL
FHHOPBeliefTreeNode* FHHOP::getOPNode(SharedPointer<BeliefWithState>& b_s)
{
	SharedPointer<belief_vector>& s = b_s->bvec;
	state_val stateidx = b_s->sval;
	bool keepLowerBound = true;

	DEBUG_TRACE(cout << "Sample::getNode stateidx " << stateidx; );
	DEBUG_TRACE(cout << " s" << endl; );
	DEBUG_TRACE(s->write(cout) << endl; );
	DEBUG_TRACE(cout << " hash: " << s->md5HashValue() << endl; );

	bool isTerminal;
	double ubVal, lbVal;

	// create a new fringe node
	FHHOPBeliefTreeNode* cn = new FHHOPBeliefTreeNode();
	int timeStamp = -1;//for initializing timeStamp of the beliefNode
	cn->s = b_s;
	cn->optimalLeafNode_AEMS2 = cn;
	cn->optimalLeafNode_FHHOP = cn;
	isTerminal = problem->getIsTerminalState(*b_s);

	onGetOPNode(this, cn, b_s);			// PAEL

	DEBUG_TRACE(cout << "current node LB " << cn->LB << endl;);
	DEBUG_TRACE(cout << "current node UB " << cn->UB << endl;);

	return cn;


}

//Assumption: cn.depth is defined already //expande node with fringe childes		PAEL
void FHHOP::expandOP(FHHOPBeliefTreeNode& cn)
{
	DEBUG_TRACE(cout << "expand" << endl; );
	DEBUG_TRACE(cn.s->bvec->write(cout) << endl; );

	// set up successors for this fringe node (possibly creating new fringe nodes)
	obs_prob_vector  opv; // outcome_prob_vector opv;
						  //state_vector *sp = new state_vector;
	SharedPointer<BeliefWithState> sp; // state_vector sp;
	obsState_prob_vector spv;  // outcome probability for values of observed state
	SharedPointer<SparseVector> jspv(new SparseVector());

	//obs_prob_vector  ospv; // outcome_prob_vector opv;

	cn.Q.resize(problem->getNumActions());

	for (Actions::iterator aIter = problem->actions->begin(); aIter != problem->actions->end(); aIter++)
	{
		int a = aIter.index();
		DEBUG_TRACE(cout << "Sample::expand a " << a << endl; );

		FHHOPBeliefTreeQEntry& Qa = cn.Q[a];
		Qa.immediateReward = problem->rewards->getReward(*cn.s, a);


		problem->getObsStateProbVector(spv, *(cn.s), a); // P(Xn|cn.s,a)
		Qa.stateOutcomes.resize(spv.size());

		DEBUG_TRACE(cout << "spv" << endl; );
		DEBUG_TRACE(spv.write(cout) << endl; );

		for (States::iterator xIter = problem->XStates->begin(); xIter != problem->XStates->end(); xIter++)
		{
			int Xn = xIter.index();

			DEBUG_TRACE(cout << "Sample::expand Xn " << Xn << endl; );

			double sprob = spv(Xn);
			if (sprob > OBS_IS_ZERO_EPS)
			{
				FHHOPBeliefTreeObsState* xe = new FHHOPBeliefTreeObsState();
				Qa.stateOutcomes[Xn] = xe;
				problem->getJointUnobsStateProbVector(*jspv, (SharedPointer<BeliefWithState>)cn.s, a, Xn);
				//problem->getStatenObsProbVectorFast(ospv, a, Xn, jspv);
				problem->getObsProbVectorFast(opv, a, Xn, *jspv); // only the joint prob is useful for later but we calculate the observation prob P(o|Xn,cn.s,a)
																  //problem->getObsProbVector(opv, cn.s, a, Xn);
				xe->outcomes.resize(opv.size());

				for (Observations::iterator oIter = problem->observations->begin(); oIter != problem->observations->end(); oIter++)
				{
					//FOR(o, opv.size())
					int o = oIter.index();

					DEBUG_TRACE(cout << "Sample::expand o " << o << endl; );

					double oprob = opv(o);
					if (oprob > OBS_IS_ZERO_EPS)
					{
						FHHOPBeliefTreeEdge* e = new FHHOPBeliefTreeEdge();
						xe->outcomes[o] = e;
						//QaXn.outcomes[o] = e;
						//e->obsProb = oprob;
						e->obsProb = oprob * sprob; // P(o,Xn|cn.s,a) = P(Xn|cn.s,a) * P(o|Xn,cn.s,a)
													//e->nextState = getNode(problem->getNextBeliefStvalFast(sp, a, o, Xn, jspv));
						sp = (problem->beliefTransition->nextBelief2(cn.s, a, o, Xn, jspv));
						sp->bvec->finalize();
						e->nextState = getOPNode(sp);
						e->nextState->parent = &cn;
						e->nextState->depth = cn.depth + 1;
						e->nextState->obsProbProduct = e->obsProb * cn.obsProbProduct;

						// for debug	PAEL
						e->nextState->optA = a;
						e->nextState->optO = o;

						// OPMDP contribution
						e->nextState->contribution_AEMS2 = e->nextState->obsProbProduct * pow(problem->discount, e->nextState->depth - this->depthOffset) * (e->nextState->UB - e->nextState->LB);
						// contribution_child = ProbProduct_child* gamma^(depth_child - offset)* (UB_child-LB_child)	PAEL
						// depth offset comes from the next iteration when we are reusing the tree.
						// parent contribution (not updated value is needed for deltaH in both cases)
						e->nextState->parentContribution_AEMS2 = cn.contribution_AEMS2;

						DEBUG_TRACE(cout << "Sample::expand e->nextState belief " << endl; );
						DEBUG_TRACE(e->nextState->s->bvec->write(cout) << endl; );

					}
					else
					{
						xe->outcomes[o] = NULL;
					}
				}
			}
			else
			{
				Qa.stateOutcomes[Xn] = NULL;
			}
		}
		Qa.ubVal = CB_QVAL_UNDEFINED;
	}
	//numStatesExpanded++;
}

// prepare the root as a fringe node.
FHHOPBeliefTreeNode* FHHOP::initOPRoot(void)
{
	//INIT BOUNDS
	initialize(this->problem);

	// INIT ROOT code
	belief_vector rootpv;
	FHHOPBeliefTreeNode* ret = NULL;
	copy(rootpv, *(problem->initialBeliefX));

	FOR(r, rootpv.size())
	{
		double rprob = rootpv(r); // .data[r].value;
		if (rprob > OBS_IS_ZERO_EPS)
		{
			SharedPointer<BeliefWithState>  thisRootb_s(new BeliefWithState());
			copy(*thisRootb_s->bvec, *problem->getInitialBeliefY(r));
			thisRootb_s->bvec->finalize();
			thisRootb_s->sval = r;

			FHHOPBeliefTreeNode* cn = FHHOP::getOPNode(thisRootb_s);

			cn->depth = 0;
			cn->obsProbProduct = 1;
			cn->parent = NULL;
			cn->parentContribution_AEMS2 = 0.0;
			cn->parentContribution_FHHOP = 0.0; 
			// initial contribution for both heuristics should be equal e(r_root) 
			// but we can use simple 1 it wont influence further calculations
			cn->contribution_AEMS2 = 1; 
			cn->contribution_FHHOP = 1;
			// delta LB and UB for root is 0 so the first root selection wont mess up the Ii value. 
			cn->deltaLB = 0.0;
			cn->deltaUB = 0.0;

			cn->optimalLeafNode_AEMS2 = cn;
			cn->optimalLeafNode_FHHOP = cn;
//			cn->entrptySum = cn->s->bvec->getEntropy();  // is this the initial value for b(s)*log(b(s)) ?
//			cn->entrptySum = 0;
			ret = cn;
		}

	}

	assert(ret && "there is no initial state given in the model");

	return ret;

}


void FHHOP::progressiveIncreasePolicyInteval(int& numPolicies)
{
	if (numPolicies == 0)
	{
		this->solverParams->interval *= 10;
		numPolicies++;

	}
	else
	{
		if (numPolicies == 5)
		{
			this->solverParams->interval *= 5;
		}
		else if (numPolicies == 10)
		{
			this->solverParams->interval *= 2;
		}
		else if (numPolicies == 15)
		{
			this->solverParams->interval *= 4;
		}

		numPolicies++;
	}
}

FHHOP::FHHOP(SharedPointer<MOMDP> problem, SolverParams * solverParams)
{
	this->problem = problem;
	this->solverParams = solverParams;
	beliefForest = new BeliefForest();
	this->firstInitialization = true;
	this->I_AEMS2 = 0;
	this->I_FHHOP = 0;
	this->N_AEMS2 = 0;
	this->N_FHHOP = 0;

	this->depthOffset = 0;
	//	sampleEngine = new SampleBPOP();						// no sampling needed PAEL::16.3.15
	//	((SampleBPOP*)sampleEngine)->setup(problem, this);
	//beliefForest->setup(problem, this->sampleEngine, &this->beliefCacheSet);
	//numBackups = 0;

	this->fileTreePrint.open("treeOP.txt", ios::out | ios::trunc);
	this->fileTreePrint.close();
}

FHHOP::~FHHOP(void)
{
}

FHHOPBeliefTreeNode* chooseLeaf(FHHOPBeliefTreeNode* root, double& I_AEMS2, double& I_FHHOP, int& N_AEMS2, int& N_FHHOP)
{

	REAL_VALUE _aems2, _fhhop;
	_aems2 = ( (I_AEMS2 + 1)/(N_AEMS2 + 1) ) * root->contribution_AEMS2;
	_fhhop = ((I_FHHOP + 1)/(N_FHHOP +1) ) * root->contribution_FHHOP;
//	cout << "CuHbu " << _aems2 << " ClHbl " << _fhhop << endl;
	if (_aems2 > _fhhop)
	{
		I_AEMS2 = I_AEMS2 + root->deltaLB + root->deltaUB;
		N_AEMS2++;

		return root->optimalLeafNode_AEMS2;
	}
	else
	{
		// if root is leaf delta Lb UB ??
		I_FHHOP = I_FHHOP + root->deltaLB + root->deltaUB;;
		N_FHHOP++;

		return root->optimalLeafNode_FHHOP;
	}
}

// get first and second best action based on LB
void getFirstSecondA(FHHOPBeliefTreeNode* b, int & f, int & s)
{
	double max1 = -99e+20, max2 = -99e+20;
	int _f, _s;
	FOR(a, b->getNodeNumActions())
	{

		FHHOPBeliefTreeQEntry & Qa = b->Q[a];

		if (Qa.lbVal > max1)
		{
			max2 = max1;
			_s = _f;

			max1 = Qa.lbVal;
			_f =a;
		}
		else{
			if (Qa.lbVal > max2)
			{
				max2 = Qa.lbVal;
				_s =a;
			}
		}
	}
	f = _f;
	s = _s;

//	cout << f << ":" << max1 << "  " << s << ":" << max2 << endl;
}

dataFHHOP FHHOP::selectFHHOP(FHHOPBeliefTreeNode* b, REAL_VALUE h, bool second, double gamma)
{
	dataFHHOP H_l, Hl_tmp;
	bool second_new = second;
	H_l.H = -99e+20;
	H_l.l = NULL;

	if (b->isFringe()) {
		if (!second) { // take it out
			H_l.H = -99e+20;
			H_l.l = b;
		} else {
			H_l.H = h * (b->UB - b->LB);
			H_l.l = b;			
		}
	} else {
		int firstA, secondA;
		getFirstSecondA(b, firstA, secondA);
		FOR(a, b->getNodeNumActions())
		{
			if (a != firstA && a != secondA)
				continue;
			else if (a == secondA && second) {
				continue;
			} else if (a == secondA && !second) {
				second_new = true;
			}

			FHHOPBeliefTreeQEntry & Qa = b->Q[a];
			FOR(xn, Qa.getNumStateOutcomes())
			{
				FHHOPBeliefTreeObsState* Qax = Qa.stateOutcomes[xn];
				if (Qax)
				{
					FOR(o, Qax->getNumOutcomes())
					{
					FHHOPBeliefTreeEdge *e = Qax->outcomes[o];
						if (e) {
							h = h*gamma*e->obsProb;
							Hl_tmp = selectFHHOP(e->nextState, h, second_new, gamma);

							if (Hl_tmp.H > H_l.H)
							{
								H_l.H =Hl_tmp.H;
								H_l.l =Hl_tmp.l;
							}
						}
					}
				}
			}
		}
		//return H_l;
	}
	return H_l;
}


void FHHOP::forwardBackup_FHHOP(FHHOPBeliefTreeNode* cn, double gamma)
{
	dataFHHOP selected;

	selected = selectFHHOP(cn, 1, 0, gamma);

	cn->contribution_FHHOP = selected.H;
	cn->optimalLeafNode_FHHOP = selected.l;

}

void FHHOP::generateTree(SharedPointer<MOMDP> problem, int action, int observation, int Xn)
{
	try
	{
		int OPbudget = this->solverParams->OPbudget; // budget;
		FHHOPBeliefTreeNode* myRoot = this->solutionRootNode;

		if (0 < this->solverParams->simNum)
		{
			uint64 Tbegin, Tdiff;
			Tbegin = GetTimeMs64_OP();
			Tdiff = 0;
			int nrBudget = 0;
			// not yet implemented the FHHOP leaf selection
			while (Tdiff <= this->solverParams->simNum)
			{
				FHHOPBeliefTreeNode* nodeToExpand =  chooseLeaf(myRoot, this->I_AEMS2, this->I_FHHOP, this->N_AEMS2, this->N_FHHOP);
				expandPrepare(nodeToExpand);
				// this solves bound backup and H_AEMS2
				backupOPNode(nodeToExpand);
				// this is forward backup of H_FHHOP
				forwardBackup_FHHOP(myRoot, problem->discount);
				nrBudget++;
				Tdiff =  GetTimeMs64_OP() - Tbegin;
			}
		}
		else
		{
			for (int i = 0; i < OPbudget; i++)
			{
//				cout << "*";
				FHHOPBeliefTreeNode* nodeToExpand =  chooseLeaf(myRoot, this->I_AEMS2, this->I_FHHOP, this->N_AEMS2, this->N_FHHOP);
				expandPrepare(nodeToExpand);
				// this solves bound backup and H_AEMS2
				backupOPNode(nodeToExpand);
				// this is forward backup of H_FHHOP
				forwardBackup_FHHOP(myRoot, problem->discount);

			}
		}

		myRoot = NULL;
	}
	catch (bad_alloc &e)
	{
		cout << "Memory limit reached, trying to write out policy" << endl;
	}
}


void FHHOP::print()
{
	if (numBackups / CHECK_INTERVAL > printIndex)
	{
		printIndex++;
		//print time now
		alwaysPrint();
	}
}

void FHHOP::alwaysPrint()
{
	//struct tms now;
	//float utime, stime;
	//long int clk_tck = sysconf(_SC_CLK_TCK);

	//print time now
	//times(&now);
	double currentTime = 0;
	if (this->solverParams->interval > 0)
	{
		currentTime = elapsed + lapTimer.elapsed();
	}
	else
	{
		currentTime = runtimeTimer.elapsed();
	}
	//printf("%.2fs ", currentTime);
	cout.precision(6);
	cout << " "; cout.width(8); cout << left << currentTime;

	//print current trial number, num of backups
	//int numTrials = ((SampleBPOP*)sampleEngine)->numTrials;
	//printf("#Trial %d ",numTrials);
//	cout.width(7); cout << left << numTrials << " ";
	//printf("#Backup %d ", numBackups);
	cout.width(8); cout << left << numBackups << " ";
	//print #alpha vectors
	//print precision

	//ADD SYLTAG
	//assume we can estimate lb and ub at the global root
	//by cycling through all the roots to find their bounds
	double lb = 0, ub = 0, width = 0;

	BeliefForest& globalRoot = *(sampleEngine->getGlobalNode());
	FOR(r, globalRoot.getGlobalRootNumSampleroots())
	{
		SampleRootEdge* eR = globalRoot.sampleRootEdges[r];
		if (eR)
		{
			BeliefTreeNode & sn = *eR->sampleRoot;
			double lbVal = beliefCacheSet[sn.cacheIndex.sval]->getRow(sn.cacheIndex.row)->LB;
			double ubVal = beliefCacheSet[sn.cacheIndex.sval]->getRow(sn.cacheIndex.row)->UB;
			lb += eR->sampleRootProb * lbVal;
			ub += eR->sampleRootProb * ubVal;
			width += eR->sampleRootProb * (ubVal - lbVal);
		}
	}

	//REMOVE SYLTAG
	//cacherow_stval rootIndex = sampleEngine->getRootNode()->cacheIndex;
	//double lb = bounds->boundsSet[rootIndex.sval]->beliefCache->getRow(rootIndex.row)->LB;
	//double ub = bounds->boundsSet[rootIndex.sval]->beliefCache->getRow(rootIndex.row)->UB;

	//printf("[%f,%f],", lb, ub);
	cout.width(10); cout << left << lb << " ";
	cout.width(10); cout << left << ub << " ";

	//print precision
	double precision = width; // ub - lb;   //MOD SYLTAG
							  //printf("%f, ", precision);
	cout.width(11);
	cout << left << precision << " ";
	int numAlphas = 0;
	FOR(setIdx, beliefCacheSet.size())
	{
		numAlphas += (int)lowerBoundSet->set[setIdx]->planes.size();
	}

	//printf("#Alphas %d ", numAlphas);			//SYLTEMP FOR EXPTS
	cout.width(9); cout << left << numAlphas;

	//print #belief nodes
	//printf("#Beliefs %d", sampleEngine->numStatesExpanded);
	cout.width(9); cout << left << sampleEngine->numStatesExpanded;

	//printf("#alphas %d", (int)bounds->alphaPlanePool->planes.size());
	printf("\n");

}

void FHHOP::logFilePrint(int index)
{
	//struct tms now;
	//float utime, stime;
	//long int clk_tck = sysconf(_SC_CLK_TCK);

	//print time now
	//times(&now);

	FILE *fp = fopen("solve.log", "a");
	if (!fp) {
		cerr << "can't open logfile\n";
		exit(1);
	}


	fprintf(fp, "%d ", index);

	//print current trial number, num of backups
	//int numTrials = ((SampleBPOP*)sampleEngine)->numTrials;
	//int numBackups = numBackups;
//	fprintf(fp, "%d ", numTrials); 			//SYLTEMP FOR EXPTS
											//printf("#Trial %d, #Backup %d ",numTrials, numBackups);

											//print #alpha vectors
	int numAlphas = 0;
	FOR(setIdx, beliefCacheSet.size())
	{
		//cout << " p : " << setIdx << " : " << 	 (int)bounds->boundsSet[setIdx]->alphaPlanePool->planes.size();
		numAlphas += (int)lowerBoundSet->set[setIdx]->planes.size();
	}

	fprintf(fp, "%d ", numAlphas);			//SYLTEMP FOR EXPTS

	double currentTime = 0;
	if (this->solverParams->interval > 0)
	{
		//utime = ((float)(now.tms_utime-last.tms_utime))/clk_tck;
		//stime = ((float)(now.tms_stime-last.tms_stime))/clk_tck;
		//currentTime = elapsed+utime+stime;
		currentTime = elapsed + lapTimer.elapsed();
		fprintf(fp, "%.2f ", currentTime);			//SYLTEMP FOR EXPTS
													//printf("<%.2fs> ", currentTime);
	}
	else {
		//utime = ((float)(now.tms_utime-start.tms_utime))/clk_tck;
		//stime = ((float)(now.tms_stime-start.tms_stime))/clk_tck;
		//currentTime = utime+stime;
		currentTime = runtimeTimer.elapsed();
		fprintf(fp, "%.2f ", currentTime);		//SYLTEMP FOR EXPTS
												//printf("<%.2fs> ", currentTime);
	}

	fprintf(fp, "\n");

	fclose(fp);
}

bool FHHOP::stopNow() {
	bool stop = false;
	return stop;
}

void FHHOP::writeIntermediatePolicyTraceToFile(int trial, double time, const string& outFileName, string problemName)
{
	stringstream newFileNameStream;
	string outputBasename = GlobalResource::parseBaseNameWithPath(outFileName);
	newFileNameStream << outputBasename << "_" << trial << "_" << time << ".policy";
	string newFileName = newFileNameStream.str();
	cout << "Writing policy file: " << newFileName << endl;
	writePolicy(newFileName, problemName);
}

BeliefTreeNode& FHHOP::getMaxExcessUncRoot(BeliefForest& globalroot)
{

	double maxExcessUnc = -99e+20;
	int maxExcessUncRoot = -1;
	double width;
	double lbVal, ubVal;

	return *(globalroot.sampleRootEdges[maxExcessUncRoot]->sampleRoot);

}

void FHHOP::initialize(SharedPointer<MOMDP> problem)
{
	printIndex = 0; // reset printing counter

	int xStateNum = problem->XStates->size();
	beliefCacheSet.resize(xStateNum);
	lbDataTableSet.resize(xStateNum);
	ubDataTableSet.resize(xStateNum);

	for (States::iterator iter = problem->XStates->begin(); iter != problem->XStates->end(); iter++)
	{
		beliefCacheSet[iter.index()] = new BeliefCache();
		lbDataTableSet[iter.index()] = new IndexedTuple<AlphaPlanePoolDataTuple>();
		ubDataTableSet[iter.index()] = new IndexedTuple<BeliefValuePairPoolDataTuple>();
	}

	initializeUpperBound(problem);

	upperBoundSet->setBeliefCache(beliefCacheSet);
	upperBoundSet->setDataTable(ubDataTableSet);

	initializeLowerBound(problem);
	lowerBoundSet->setBeliefCache(beliefCacheSet);
	lowerBoundSet->setDataTable(lbDataTableSet);

	initializeBounds(this->solverParams->targetPrecision);

}

void FHHOP::initializeUpperBound(SharedPointer<MOMDP> problem)
{
	upperBoundSet = new BeliefValuePairPoolSet(upperBoundBackup);
	upperBoundSet->setProblem(problem);
	upperBoundSet->setSolver(this);
	upperBoundSet->initialize();
	upperBoundSet->appendOnBackupHandler(&FHHOP::onUpperBoundBackup);
	((BackupBeliefValuePairMOMDP*)upperBoundBackup)->boundSet = upperBoundSet;
}

void FHHOP::initializeLowerBound(SharedPointer<MOMDP> problem)
{
	lowerBoundSet = new AlphaPlanePoolSet(lowerBoundBackup);
	lowerBoundSet->setProblem(problem);
	lowerBoundSet->setSolver(this);
	lowerBoundSet->initialize();
	lowerBoundSet->appendOnBackupHandler(&FHHOP::onLowerBoundBackup);
	lowerBoundSet->appendOnBackupHandler(&SARSOPPrune::onLowerBoundBackup);
	((BackupAlphaPlaneMOMDP*)lowerBoundBackup)->boundSet = lowerBoundSet;
}

void FHHOP::initializeBounds(double _targetPrecision)
{
	double targetPrecision = _targetPrecision * CB_INITIALIZATION_PRECISION_FACTOR;

	CPTimer heurTimer;
	heurTimer.start(); 	// for timing heuristics
	BlindLBInitializer blb(problem, lowerBoundSet);
	blb.initialize(targetPrecision);
	elapsed = heurTimer.elapsed();

	DEBUG_LOG(cout << fixed << setprecision(2) << elapsed << "s blb.initialize(targetPrecision) done" << endl; );

	heurTimer.restart();
	FastInfUBInitializer fib(problem, upperBoundSet);
	fib.initialize(targetPrecision);
	elapsed = heurTimer.elapsed();
	DEBUG_LOG(cout << fixed << setprecision(2) << elapsed << "s fib.initialize(targetPrecision) done" << endl;);

	FOR(state_idx, problem->XStates->size())
	{
		upperBoundSet->set[state_idx]->cornerPointsVersion++;	// advance the version by one so that next time get value will calculate rather than skip
	}

	//cout << "finished setting cornerPointsVersion" << endl;

	numBackups = 0;
}//end method initialize

void FHHOP::writePolicy(string fileName, string problemName)
{
	writeToFile(fileName, problemName);
}

void FHHOP::writeToFile(const std::string& outFileName, string problemName)
{
	lowerBoundSet->writeToFile(outFileName, problemName);

}

void FHHOP::printHeader() {
	cout << endl;
	printDivider();
	cout << " Time   |#Trial |#Backup |LBound    |UBound    |Precision  |#Alphas |#Beliefs  " << endl;
	printDivider();
}

void FHHOP::printDivider() {
	cout << "-------------------------------------------------------------------------------" << endl;
}
