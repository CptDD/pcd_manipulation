#include "treeHandler.h"


void treeHandler::solve(SharedPointer<MOMDP> problem)
{}
void treeHandler::writePolicy(string fileName, string problemName)
{}
int treeHandler::getOptimalAction(SharedPointer<MOMDP> problem, int action, int observation, int Xn)
{
	return 0;
}

treeHandler::treeHandler(SharedPointer<MOMDP> problem, SolverParams * solverParams)
{
	this->problem = problem;
	this->solverParams = solverParams;
	this->maxDepth = this->solverParams->simNum;
	this->deletedNodes = 0;
}

treeHandler::~treeHandler()
{
}

double getReward(SharedPointer<MOMDP> problem, const BeliefWithState& belst, int action)
{
	//const SparseMatrix rewMat = problem->getRewardMatrix(belst.sval);
	const SharedPointer<SparseMatrix>  rewMat = problem->rewards->getMatrix(belst.sval);
	return inner_prod_column(*rewMat, action, *belst.bvec);
}

int randomPDF(SharedPointer<DenseVector> pdf)
{
	int res = -1;
	double randomNr = (double)(rand() % 101) / 100.0;
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

void treeHandler::buildFullTree(int maxDepth, SharedPointer<MOMDP> problem, DHSBeliefTreeNode * cn)
{

	//cout << cn->depth << endl;

	if (cn->depth < maxDepth)
	{
		expandPrepare(cn, problem);

		FOR(Aval, cn->Q.size())
		{
			OPBeliefTreeQEntry * Qa = &cn->Q[Aval];

			FOR(Xni, Qa->getNumStateOutcomes())
			{
				OPBeliefTreeObsState * Xn = Qa->stateOutcomes[Xni];

				if (NULL != Xn)
				{
					FOR(Oval, Xn->getNumOutcomes())
					{
						OPBeliefTreeEdge *Xo = Xn->outcomes[Oval];
						if (NULL != Xo)
						{
							buildFullTree(maxDepth, problem, Xo->nextState);
							//cout << cn->depth << endl;
						}
					}
				}
			}
		}

	}
	else
	{
		//cout << ".";
	}


}

void treeHandler::expandPrepare(DHSBeliefTreeNode* cn, SharedPointer<MOMDP> problem)
{
	if (cn->isFringe())
	{
		expandOP(*cn, problem);
	}
	else
	{
		cout << "ERROR = the given node for expantion is not fringe";
	}
}

void treeHandler::expandOP(DHSBeliefTreeNode& cn, SharedPointer<MOMDP> problem)
{
	DEBUG_TRACE(cout << "expand" << endl;);
	DEBUG_TRACE(cn.s->bvec->write(cout) << endl;);

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
		DEBUG_TRACE(cout << "Sample::expand a " << a << endl;);

		OPBeliefTreeQEntry& Qa = cn.Q[a];
		Qa.immediateReward = problem->rewards->getReward(*cn.s, a);


		problem->getObsStateProbVector(spv, *(cn.s), a); // P(Xn|cn.s,a)
		Qa.stateOutcomes.resize(spv.size());

		DEBUG_TRACE(cout << "spv" << endl;);
		DEBUG_TRACE(spv.write(cout) << endl;);

		for (States::iterator xIter = problem->XStates->begin(); xIter != problem->XStates->end(); xIter++)
		{
			int Xn = xIter.index();

			DEBUG_TRACE(cout << "Sample::expand Xn " << Xn << endl;);

			double sprob = spv(Xn);
			if (sprob > OBS_IS_ZERO_EPS)
			{
				OPBeliefTreeObsState* xe = new OPBeliefTreeObsState();
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

					DEBUG_TRACE(cout << "Sample::expand o " << o << endl;);

					double oprob = opv(o);
					if (oprob > OBS_IS_ZERO_EPS)
					{
						OPBeliefTreeEdge* e = new OPBeliefTreeEdge();
						xe->outcomes[o] = e;
						//QaXn.outcomes[o] = e;
						//e->obsProb = oprob;
						e->obsProb = oprob * sprob; // P(o,Xn|cn.s,a) = P(Xn|cn.s,a) * P(o|Xn,cn.s,a)
						//e->nextState = getNode(problem->getNextBeliefStvalFast(sp, a, o, Xn, jspv));
						sp = (problem->beliefTransition->nextBelief2(cn.s, a, o, Xn, jspv));
						sp->bvec->finalize();
						e->nextState = getOPNode(sp, problem);
						e->nextState->parent = &cn;
						e->nextState->depth = cn.depth + 1;
						e->nextState->obsProbProduct = e->obsProb * cn.obsProbProduct;
						// for debug	PAEL
						e->nextState->optA = a;
						e->nextState->optO = o;

						e->nextState->contribution_LSEM = e->nextState->obsProbProduct * pow(problem->discount, e->nextState->depth /*- this->depthOffset*/) * (e->nextState->UB - e->nextState->LB);
						//contribution_child = ProbProduct_child* gamma^(depth_child - offset)* (UB_child-LB_child)	PAEL
						// depth offset comes from the next iteration when we are reusing the tree.

						DEBUG_TRACE(cout << "Sample::expand e->nextState belief " << endl;);
						DEBUG_TRACE(e->nextState->s->bvec->write(cout) << endl;);

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

DHSBeliefTreeNode* treeHandler::getOPNode(SharedPointer<BeliefWithState>& b_s, SharedPointer<MOMDP> problem)
{
	SharedPointer<belief_vector>& s = b_s->bvec;
	state_val stateidx = b_s->sval;
	bool keepLowerBound = true;

	DEBUG_TRACE(cout << "Sample::getNode stateidx " << stateidx;);
	DEBUG_TRACE(cout << " s" << endl;);
	DEBUG_TRACE(s->write(cout) << endl;);
	DEBUG_TRACE(cout << " hash: " << s->md5HashValue() << endl;);

	bool isTerminal;
	double ubVal, lbVal;

	// create a new fringe node
	DHSBeliefTreeNode* cn = new DHSBeliefTreeNode();
	int timeStamp = -1;//for initializing timeStamp of the beliefNode
	cn->s = b_s;
	cn->optimalLeafNode_LSEM = cn;
	isTerminal = problem->getIsTerminalState(*b_s);

	// No need to set LB & UB int this test! PAEL Memory free test
	//onGetOPNode(this, cn, b_s);			// PAEL

	DEBUG_TRACE(cout << "current node LB " << cn->LB << endl;);
	DEBUG_TRACE(cout << "current node UB " << cn->UB << endl;);

	return cn;

}

DHSBeliefTreeNode* treeHandler::initOPRoot(SharedPointer<MOMDP> problem)
{

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

			DHSBeliefTreeNode* cn = getOPNode(thisRootb_s, problem);

			cn->depth = 0;
			cn->obsProbProduct = 1;
			cn->optimalLeafNode_LSEM = cn;
			cn->parent = NULL;

			ret = cn;
		}

	}

	assert(ret != NULL && "there is no initial state given in the model");

	return ret;

}

DHSBeliefTreeNode * treeHandler::freeSubTree2(DHSBeliefTreeNode *cn, int A, int O, int X, bool firstLeve)
{
	if (cn->isFringe())
	{
		cn->parent = NULL;
		cn->optimalLeafNode_LSEM = NULL;
		delete cn;
		this->deletedNodes++;
	}
	else
	{
		FOR(Aval, cn->Q.size())
		{
			OPBeliefTreeQEntry * Qa = &cn->Q[Aval];

			FOR(Xni, Qa->getNumStateOutcomes())
			{
				OPBeliefTreeObsState * Xn = Qa->stateOutcomes[Xni];

				if (Xn)
				{
					FOR(Oval, Xn->getNumOutcomes())
					{
						OPBeliefTreeEdge *Xo = Xn->outcomes[Oval];
						if (Xo)
						{
							if (A == Aval && O == Oval && X == Xni && firstLeve)
							{
								Xo->nextState->parent = NULL;
								return Xo->nextState;
							}
							else
							{
								freeSubTree2(Xo->nextState, -1, -1, -1, false);
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

int treeHandler::runMemFreeTest(int maxDepth, bool partialDelete)
{
	int depthTarget = 0;
	if (0 > maxDepth)
	{
		depthTarget = this->maxDepth;		
	}
	else
		depthTarget = maxDepth;
	cout << "Memeory free test started" << endl;

	if (!this->solutionRootNode)
	{
		this->solutionRootNode = initOPRoot(this->problem);
		cout << "root init done" << endl;
	}
	
	cout << "target depth=" << depthTarget << endl;

	buildFullTree(depthTarget, this->problem, this->solutionRootNode);

	cout << "tree built till depth " << depthTarget << endl;

	int a = -1, o = -1, xn = -1;

	FOR(Aval, this->solutionRootNode->Q.size())
	{
		OPBeliefTreeQEntry*  Qa = &this->solutionRootNode->Q[Aval];

		FOR(Xni, Qa->getNumStateOutcomes())
		{
			OPBeliefTreeObsState * Xn = Qa->stateOutcomes[Xni];

			if (NULL != Xn)
			{
				FOR(Oval, Xn->getNumOutcomes())
				{
					OPBeliefTreeEdge *Xo = Xn->outcomes[Oval];
					if (NULL != Xo)
					{
						a = Aval;
						o = Oval;
						xn = Xni;
						//cout << "chosen a " << a << " o " << o << " xn " << xn << endl;
						break;
					}
				}
			}
		}
	}

	cout << "chosen branch is: a=" << a << " xn=" << xn << " o=" << o << endl;
	
	this->solutionRootNode = freeSubTree2(this->solutionRootNode, a, o, xn, partialDelete);

	cout << "memory free done, deleted nodes= " << deletedNodes << endl;

	return a;
}