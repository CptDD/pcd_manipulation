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

#include "DHS.h"
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
void DHS::solve(SharedPointer<MOMDP> problem)
{
}

DHSBeliefTreeNode * DHS::freeSubTree(DHSBeliefTreeNode *cn, int A, int O, int X, bool firstLeve)
{
	if (cn->isFringe())
	{
		cn->parent = NULL;
		cn->optimalLeafNode_AEMS2 = NULL;
		cn->optimalLeafNode_LSEM = NULL;
		delete cn;
	}
	else
	{
		FOR(Aval, cn->Q.size())
		{
			DHSBeliefTreeQEntry * Qa = &cn->Q[Aval];

			FOR(Xni, Qa->getNumStateOutcomes())
			{
				DHSBeliefTreeObsState * Xn = Qa->stateOutcomes[Xni];

				if (Xn)
				{
					FOR(Oval, Xn->getNumOutcomes())
					{
						DHSBeliefTreeEdge *Xo = Xn->outcomes[Oval];
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

int DHS::getOptimalAction(SharedPointer<MOMDP> problem, int action, int observation, int Xn)
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


	// buld optimal tree
	generateTree(problem, action, observation, Xn);

	//search optimal action
	int maxAction = -1;
	double maxLb = -99e+20;

	FOR(a, this->solutionRootNode->getNodeNumActions())
	{
		DHSBeliefTreeQEntry & Qa = this->solutionRootNode->Q[a];
		if (maxLb < Qa.lbVal)
		{
			maxLb = Qa.lbVal;
			maxAction = a;
		}
	}

	cout << "ALG SELECTION AEMS - LSEM :" << this->algAEMS2 << ":" << this->algLSEM << endl << endl;


	return maxAction;
}



// propagate lower and upper bound, aslo select besta action	PAEL
// we are not going to use the returned max action ID
int DHS::backupUb_Lb(DHSBeliefTreeNode* cn)
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

		DHSBeliefTreeQEntry & Qa = cn->Q[a];

		reward = Qa.immediateReward;
		cn_la = 0.0;
		cn_ua = 0.0;

		FOR(xn, Qa.getNumStateOutcomes())
		{
			DHSBeliefTreeObsState* Qax = Qa.stateOutcomes[xn];
			if (Qax)
				FOR(o, Qax->getNumOutcomes())
			{
				if (Qax->outcomes[o])
				{
					DHSBeliefTreeEdge *e = Qax->outcomes[o];

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

	cn->LB = maxLB;
	cn->UB = maxUB;

	return maxAction;
}

// propagate max contribution and optimal leaf node		PAEL
// we look through all the actions not only on the optimal one!
void DHS::backupContributution(DHSBeliefTreeNode* cn, int maxAction)
{
	DHSBeliefTreeQEntry & Qa = cn->Q[maxAction];

	double maxContribution_AEMS2 = -99e+20, maxContribution_LSEM = -99e+20;
	DHSBeliefTreeNode* maxContributionNode_AEMS2 = NULL;
	DHSBeliefTreeNode* maxContributionNode_LSEM = NULL;

	FOR(a, cn->getNodeNumActions())
	{
		DHSBeliefTreeQEntry & Qa = cn->Q[a];
		FOR(xn, Qa.getNumStateOutcomes())
		{
			DHSBeliefTreeObsState* Qax = Qa.stateOutcomes[xn];
			if (Qax)
				FOR(o, Qax->getNumOutcomes())
				{
				DHSBeliefTreeEdge *e = Qax->outcomes[o];
					if (e) {

						// AMES2 contribution backup only for optimal action
						if (e->nextState->contribution_AEMS2
								> maxContribution_AEMS2 && a == maxAction) {
							maxContribution_AEMS2 =
									e->nextState->contribution_AEMS2;
							maxContributionNode_AEMS2 = e->nextState;
						}
						// LSEM contribution backup for all actions
						if (e->nextState->contribution_LSEM
								> maxContribution_LSEM) {
							maxContribution_LSEM =
									e->nextState->contribution_LSEM;
							maxContributionNode_LSEM = e->nextState;
						}
					}

				}
		}
	}

	// save the leafe node that  should be expanded at next step PAEL
	cn->optimalLeafNode_AEMS2 = maxContributionNode_AEMS2->optimalLeafNode_AEMS2;
	cn->contribution_AEMS2 = maxContribution_AEMS2;

	cn->optimalLeafNode_LSEM = maxContributionNode_LSEM->optimalLeafNode_LSEM;
	cn->contribution_LSEM = maxContribution_LSEM;
}

// Backup the expanded node		// PAEL
void DHS::backupOPNode(DHSBeliefTreeNode* cn)
{
	do
	{
		// maxAction is used only in AEMS2
		// in this case we don't use it after obtaining it
		// backup the bounds and select max action
		int maxAction = backupUb_Lb(cn);

		//ssert(maxAction != -1 " the node requested for backup is a finge node!");

		// backup contribution & optimal leaf selection
		backupContributution(cn, maxAction);

		cn = cn->parent;
	} while (cn);

}

// check if the node is fringe and then expand it.		PAEL
void DHS::expandPrepare(DHSBeliefTreeNode* cn)
{
	if (cn->isFringe())
	{
		expandOP(*cn);
	}
	else
	{
		cout << "ERROR = the given node for expantion is not fringe";
	}
}

void onGetOPNode(DHS *solver, DHSBeliefTreeNode* node, SharedPointer<BeliefWithState>& belief)
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
DHSBeliefTreeNode* DHS::getOPNode(SharedPointer<BeliefWithState>& b_s)
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
	DHSBeliefTreeNode* cn = new DHSBeliefTreeNode();
	int timeStamp = -1;//for initializing timeStamp of the beliefNode
	cn->s = b_s;
	cn->optimalLeafNode_AEMS2 = cn;
	cn->optimalLeafNode_LSEM = cn;
	isTerminal = problem->getIsTerminalState(*b_s);

	onGetOPNode(this, cn, b_s);			// PAEL

	DEBUG_TRACE(cout << "current node LB " << cn->LB << endl;);
	DEBUG_TRACE(cout << "current node UB " << cn->UB << endl;);

	return cn;


}

//Assumption: cn.depth is defined already //expande node with fringe childes		PAEL
void DHS::expandOP(DHSBeliefTreeNode& cn)
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

		DHSBeliefTreeQEntry& Qa = cn.Q[a];
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
				DHSBeliefTreeObsState* xe = new DHSBeliefTreeObsState();
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
						DHSBeliefTreeEdge* e = new DHSBeliefTreeEdge();
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

						// LSEM contribution and parent contribution (not updated value is needed for deltaH in both cases)
						e->nextState->contribution_LSEM = log(problem->getBeliefSize()/log(2.0) - e->nextState->s->bvec->getEntropy() )*e->nextState->UB*(1+log(e->nextState->depth - this->depthOffset + 1))*pow(problem->discount, e->nextState->depth - this->depthOffset)*e->nextState->obsProbProduct;
						e->nextState->parentContribution_LSEM = cn.contribution_LSEM;

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
DHSBeliefTreeNode* DHS::initOPRoot(void)
{
	//INIT BOUNDS
	initialize(this->problem);

	// INIT ROOT code
	belief_vector rootpv;
	DHSBeliefTreeNode* ret = NULL;
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

			DHSBeliefTreeNode* cn = DHS::getOPNode(thisRootb_s);

			cn->depth = 0;
			cn->obsProbProduct = 1;
			cn->parent = NULL;
			cn->parentContribution_AEMS2 = 0.0;
			cn->parentContribution_LSEM = 0.0;
			cn->optimalLeafNode_AEMS2 = cn;
			cn->optimalLeafNode_LSEM = cn;
			cn->contribution_AEMS2 = abs(cn->UB - cn->LB);
			cn->contribution_LSEM =log(problem->getBeliefSize()/log(2.0) - cn->s->bvec->getEntropy() )*cn->UB*(1+log(1))*pow(problem->discount, 0)*1;
//			cn->entrptySum = cn->s->bvec->getEntropy();  // is this the initial value for b(s)*log(b(s)) ?
//			cn->entrptySum = 0;
			ret = cn;
		}

	}

	assert(ret && "there is no initial state given in the model");

	return ret;

}


void DHS::progressiveIncreasePolicyInteval(int& numPolicies)
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

DHS::DHS(SharedPointer<MOMDP> problem, SolverParams * solverParams)
{
	this->problem = problem;
	this->solverParams = solverParams;
	beliefForest = new BeliefForest();
	this->firstInitialization = true;

	this->depthOffset = 0;
	//	sampleEngine = new SampleBPOP();						// no sampling needed PAEL::16.3.15
	//	((SampleBPOP*)sampleEngine)->setup(problem, this);
	//beliefForest->setup(problem, this->sampleEngine, &this->beliefCacheSet);
	//numBackups = 0;

	this->fileTreePrint.open("treeOP.txt", ios::out | ios::trunc);
	this->fileTreePrint.close();
	this->algAEMS2 = 0;
	this->algLSEM = 0;
}

DHS::~DHS(void)
{
}

DHSBeliefTreeNode* chooseLeaf(DHSBeliefTreeNode* root, double gamma, bool& algAEMS)
{
	REAL_VALUE deltaH_aems2, deltaH_slem;

	deltaH_aems2 = abs((root->optimalLeafNode_AEMS2->contribution_AEMS2/gamma - root->optimalLeafNode_AEMS2->parentContribution_AEMS2))/root->optimalLeafNode_AEMS2->parentContribution_AEMS2;
	deltaH_slem = abs((root->optimalLeafNode_LSEM->contribution_LSEM/gamma - root->optimalLeafNode_LSEM->parentContribution_LSEM))/root->optimalLeafNode_LSEM->parentContribution_LSEM;
	//cout << "S:A "<< deltaH_slem  << ":" << deltaH_aems2  <<", \n";

	if (deltaH_slem > deltaH_aems2)

	{
		algAEMS = false;
//
		return root->optimalLeafNode_LSEM;

	}
	else
	{
		algAEMS = true;
//		cout << "AEMS2 "<< deltaH_slem  << ">" << deltaH_aems2  << ", \n";
		return root->optimalLeafNode_AEMS2;

	}

}

void DHS::generateTree(SharedPointer<MOMDP> problem, int action, int observation, int Xn)
{
	try
	{
		int OPbudget = this->solverParams->OPbudget; // budget;
		DHSBeliefTreeNode* myRoot = this->solutionRootNode;
		bool algAEMS;
		if (0 < this->solverParams->simNum)
		{
			uint64 Tbegin, Tdiff;
			Tbegin = GetTimeMs64_OP();
			Tdiff = 0;
			int nrBudget = 0;
			// not yet implemented the DHS leaf selection
			while (Tdiff <= this->solverParams->simNum)
			{
				DHSBeliefTreeNode* nodeToExpand = chooseLeaf(myRoot, problem->discount, algAEMS);
				if (algAEMS)
				{
					this->algAEMS2++;
				}
				else
				{
					this->algLSEM++;
				}

				expandPrepare(nodeToExpand);
				backupOPNode(nodeToExpand);
				nrBudget++;
				Tdiff =  GetTimeMs64_OP() - Tbegin;
			}
		}
		else
		{
			for (int i = 0; i < OPbudget; i++)
			{
				DHSBeliefTreeNode* nodeToExpand =  chooseLeaf(myRoot, problem->discount, algAEMS);
				if (algAEMS)
				{
					this->algAEMS2++;
				}
				else
				{
					this->algLSEM++;
				}

				expandPrepare(nodeToExpand);
				backupOPNode(nodeToExpand);
			}
		}

		myRoot = NULL;
	}
	catch (bad_alloc &e)
	{
		cout << "Memory limit reached, trying to write out policy" << endl;
	}
}


void DHS::print()
{
	if (numBackups / CHECK_INTERVAL > printIndex)
	{
		printIndex++;
		//print time now
		alwaysPrint();
	}
}

void DHS::alwaysPrint()
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

void DHS::logFilePrint(int index)
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

bool DHS::stopNow() {
	bool stop = false;
	return stop;
}

void DHS::writeIntermediatePolicyTraceToFile(int trial, double time, const string& outFileName, string problemName)
{
	stringstream newFileNameStream;
	string outputBasename = GlobalResource::parseBaseNameWithPath(outFileName);
	newFileNameStream << outputBasename << "_" << trial << "_" << time << ".policy";
	string newFileName = newFileNameStream.str();
	cout << "Writing policy file: " << newFileName << endl;
	writePolicy(newFileName, problemName);
}

BeliefTreeNode& DHS::getMaxExcessUncRoot(BeliefForest& globalroot)
{

	double maxExcessUnc = -99e+20;
	int maxExcessUncRoot = -1;
	double width;
	double lbVal, ubVal;

	return *(globalroot.sampleRootEdges[maxExcessUncRoot]->sampleRoot);

}

void DHS::initialize(SharedPointer<MOMDP> problem)
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

void DHS::initializeUpperBound(SharedPointer<MOMDP> problem)
{
	upperBoundSet = new BeliefValuePairPoolSet(upperBoundBackup);
	upperBoundSet->setProblem(problem);
	upperBoundSet->setSolver(this);
	upperBoundSet->initialize();
	upperBoundSet->appendOnBackupHandler(&DHS::onUpperBoundBackup);
	((BackupBeliefValuePairMOMDP*)upperBoundBackup)->boundSet = upperBoundSet;
}

void DHS::initializeLowerBound(SharedPointer<MOMDP> problem)
{
	lowerBoundSet = new AlphaPlanePoolSet(lowerBoundBackup);
	lowerBoundSet->setProblem(problem);
	lowerBoundSet->setSolver(this);
	lowerBoundSet->initialize();
	lowerBoundSet->appendOnBackupHandler(&DHS::onLowerBoundBackup);
	lowerBoundSet->appendOnBackupHandler(&SARSOPPrune::onLowerBoundBackup);
	((BackupAlphaPlaneMOMDP*)lowerBoundBackup)->boundSet = lowerBoundSet;
}

void DHS::initializeBounds(double _targetPrecision)
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

void DHS::writePolicy(string fileName, string problemName)
{
	writeToFile(fileName, problemName);
}

void DHS::writeToFile(const std::string& outFileName, string problemName)
{
	lowerBoundSet->writeToFile(outFileName, problemName);

}

void DHS::printHeader() {
	cout << endl;
	printDivider();
	cout << " Time   |#Trial |#Backup |LBound    |UBound    |Precision  |#Alphas |#Beliefs  " << endl;
	printDivider();
}

void DHS::printDivider() {
	cout << "-------------------------------------------------------------------------------" << endl;
}
