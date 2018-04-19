#ifndef FHHOP_H
#define FHHOP_H

#include <list>
#include "PointBasedAlgorithm.h"
#include "Bound.h"
#include "BoundSet.h"
#include "AlphaPlane.h"
#include "BeliefCache.h"
#include "BeliefForest.h"
#include "BinManager.h"
#include "BinManagerSet.h"
//#include "SampleBPOP.h"
#include "AlphaPlanePool.h"
#include "AlphaPlanePoolSet.h"
#include "BeliefValuePair.h"
#include "BeliefValuePairPool.h"
#include "BeliefValuePairPoolSet.h"
#include "AlphaPlaneMaxMeta.h"
#include "Tuple.h"
#include "FacmodelStructs.h"
#include "GlobalResource.h"

// newly added for tre generation	PAEL
#include "FHHOPBeliefTreeNode.h"
#include <iostream>
#include <fstream>
#include <iomanip>

#define EPSILON 0.15

using namespace std;
namespace momdp
{

typedef struct{
	FHHOPBeliefTreeNode* l;
	REAL_VALUE H;
} dataFHHOP;

	class BinManager;
	class SARSOPPrune;

#define CHECK_INTERVAL 50

#undef DEBUG_TRACE_ON

	// comented cuz it is duplicat in SARSOP PAEL::16.03.15
	class FHHOPAlphaPlaneTuple :public Tuple
	{
	public:
		int certed;//for recording the usage as max plane at some belief points
		vector<BeliefTreeNode*> certifiedBeliefs;	// only non-fringe nodes
		vector<int> certifiedBeliefTimeStamps;
		vector<AlphaPlaneMaxMeta*> maxMeta; // all nodes
		bool sanityMax;

	};



	class FHHOP : public PointBasedAlgorithm
	{
	private:
		FHHOPBeliefTreeNode * solutionRootNode;
		unsigned int depthOffset;
		bool firstInitialization;

		double I_AEMS2, I_FHHOP;
		int N_AEMS2, N_FHHOP;

	public:
		ofstream fileTreePrint;

//		FHHOPBeliefTreeNode* FHHOP::chooseLeaf(FHHOPBeliefTreeNode* root);

		FHHOP(SharedPointer<MOMDP> problem, SolverParams * solverParams);
		virtual ~FHHOP(void);

		// FHHOP leaf selection based on deltaH_aems2 and deltaH_slem
		//FHHOPBeliefTreeNode* chooseLeaf(FHHOPBeliefTreeNode* root, double gamma);

		dataFHHOP selectFHHOP(FHHOPBeliefTreeNode* b, REAL_VALUE h, bool second, double gamma);
		// new functions to generate the FRINGE childs of a node	PAEL
		void expandPrepare(FHHOPBeliefTreeNode* cn);
		FHHOPBeliefTreeNode* getOPNode(SharedPointer<BeliefWithState>& b_s);
		void expandOP(FHHOPBeliefTreeNode& cn);
		FHHOPBeliefTreeNode* initOPRoot(void);

		virtual int getOptimalAction(SharedPointer<MOMDP> problem, int action, int observation, int Xn);
		//void OPMDP::buildFullTree(int maxDepth, SharedPointer<MOMDP> problem, OPBeliefTreeNode * cn);

		void backupOPNode(FHHOPBeliefTreeNode* cn);
		void backupContributution_AEMS2(FHHOPBeliefTreeNode* cn, int maxAction);
		void forwardBackup_FHHOP(FHHOPBeliefTreeNode* cn, double gamma);

		int backupUb_Lb(FHHOPBeliefTreeNode* cn);

		BeliefValuePairPoolSet *upperBoundSet;
		AlphaPlanePoolSet *lowerBoundSet;

		BinManagerSet* binManagerSet;

		SARSOPPrune* pruneEngine;

		Backup<BeliefValuePair> *upperBoundBackup;
		Backup<AlphaPlane> *lowerBoundBackup;

		vector<IndexedTuple<AlphaPlanePoolDataTuple> *> lbDataTableSet;
		vector<IndexedTuple<BeliefValuePairPoolDataTuple> *> ubDataTableSet;



		void generateTree(SharedPointer<MOMDP> problem, int action, int observation, int Xn);
		//void solve(SharedPointer<MOMDP> problem, int budget);		//PAEL
		virtual void writePolicy(string fileName, string problemName);
		void writeToFile(const std::string& outFileName, string problemName);

		virtual void solve(SharedPointer<MOMDP> problem);
		FHHOPBeliefTreeNode* freeSubTree(FHHOPBeliefTreeNode *cn, int A, int O, int X, bool firstLevel);


		BeliefTreeNode* sample();
		void backup(BeliefTreeNode* node);

		// Callback functions, must be static, or else it will not match the required signature due the member function's implicit "this" pointer
		static void onLowerBoundBackup(PointBasedAlgorithm *solver, BeliefTreeNode * node, SharedPointer<AlphaPlane> backupResult)
		{
			// updating certs, etc
		}

		static void onUpperBoundBackup(PointBasedAlgorithm *solver, BeliefTreeNode * node, SharedPointer<BeliefValuePair> backupResult)
		{
		}
		// print out the tree till a well defined depth		PAEL
		void printTreeOP(FHHOPBeliefTreeNode *root, int depth, int currentDepth, REAL_VALUE Prob, int action, int observation)
		{
			if (depth >= root->depth )
			{
				//ofstream myfile;
				//myfile.open("treeOP.txt", ios::out | ios::app);
				fileTreePrint << action << " " << observation << " " << root->s->sval << " ";
				root->s->bvec->write(fileTreePrint) << " ";
				fileTreePrint << std::setprecision(5)
					<< root->LB << " " << root->UB << " ";
				fileTreePrint << root->depth << " " << std::setprecision(5) << Prob << " " << endl;
				//upperBoundSet->set[root->s->sval]->cornerPoints.write(fileTreePrint) << endl;

				FOR(Aval, root->Q.size())
				{
					FHHOPBeliefTreeQEntry * Qa = &root->Q[Aval];

					FOR(Xni, Qa->getNumStateOutcomes())
						{
						FHHOPBeliefTreeObsState * Xn = Qa->stateOutcomes[Xni];

							if (NULL != Xn)
							{
								FOR(Oval, Xn->getNumOutcomes())
								{
									FHHOPBeliefTreeEdge *Xo = Xn->outcomes[Oval];
									if (NULL != Xo)
										printTreeOP(Xo->nextState, depth, currentDepth + 1, Xo->obsProb, Aval, Oval);
									//else
										//myfile.close();
								}
							}
							//myfile.close();
						}

				}
				//myfile.close();
			}
		}

		///////////TESTING
		/*
		void onGetOPNode(PointBasedAlgorithm *solver, OPBeliefTreeNode* node, SharedPointer<BeliefWithState>& belief)
		{
			OPMDP *opSolver = (OPMDP *)solver;
			int stateidx = belief->sval;
//			int row = node->cacheIndex.row;
			int timeStamp = opSolver->numBackups;

			// SARSOP Bin Manager related
//			sarsopSolver->binManagerSet->binManagerSet[stateidx]->binManagerDataTable.set(row).binned = false;

			// TODO: fix this bug, UB_ACTION is set by backup, but if a node is allocated and sampled before backup, UB_ACTION is not defined
//			sarsopSolver->upperBoundSet->set[stateidx]->dataTable->set(row).UB_ACTION = 0;


			list<SharedPointer<AlphaPlane> >* alphas = new list<SharedPointer<AlphaPlane> >();
//			sarsopSolver->lowerBoundSet->set[stateidx]->dataTable->set(row).ALPHA_PLANES = alphas;

			// TODO:: fix it
			SharedPointer<AlphaPlane>alpha = opSolver->lowerBoundSet->getValueAlpha(belief);
			//REAL_VALUE lbVal = sarsopSolver->lowerBoundSet->getValue(belief);
			REAL_VALUE lbVal = inner_prod(*alpha->alpha, *belief->bvec);
			//REAL_VALUE lbVal = bounds->getLowerBoundValue(b_s, &alpha);

			OPMDPAlphaPlaneTuple *dataAttachedToAlpha = (OPMDPAlphaPlaneTuple *)(alpha->solverData);

			REAL_VALUE ubVal = opSolver->upperBoundSet->getValue(belief);

			node->LB = lbVal;
			node->UB = ubVal;
//			sarsopSolver->lowerBoundSet->set[stateidx]->dataTable->set(row).ALPHA_TIME_STAMP = timeStamp;

			if (timeStamp != -1)
			{
				//assert(solver->beliefCacheSet[stateidx]->getRow( row)->isFringe );
				DEBUG_TRACE("getNode timeStamp!=-1");
				if (true)//(!hasMaxMetaAt(alpha, node->cacheIndex.row))	<- original // assume that the alpha is from the correct boundsSet[]  //PAEL
				{
					DEBUG_TRACE("!hasMaxMetaAt");
					AlphaPlaneMaxMeta* newMax = new AlphaPlaneMaxMeta();
//					newMax->cacheIndex = node->cacheIndex.row;
					newMax->lastLB = lbVal;
					newMax->timestamp = GlobalResource::getInstance()->getTimeStamp();
					dataAttachedToAlpha->maxMeta.push_back(newMax);
				}
			}

			opSolver = NULL;
		}
		*/

		// Initialization

		void initialize(SharedPointer<MOMDP> problem);
		void initSampleEngine(SharedPointer<MOMDP> problem);
		void initializeUpperBound(SharedPointer<MOMDP> problem);
		void initializeLowerBound(SharedPointer<MOMDP> problem);
		void initializeBounds(double _targetPrecision);

		// Alpha Vector Related
		static bool hasMaxMetaAt(SharedPointer<AlphaPlane>alpha, int index)
		{
			FHHOPAlphaPlaneTuple *attachedData = (FHHOPAlphaPlaneTuple *)alpha->solverData;
			FOREACH(AlphaPlaneMaxMeta*, entry, attachedData->maxMeta)
			{
				if ((*entry)->cacheIndex == index)
				{
					return true;
				}
			}
			return false;
		}

		// Timers
		CPTimer	runtimeTimer;
		CPTimer lapTimer;

		double elapsed;

		// Print
		int printIndex;

		void alwaysPrint();
		void printHeader();
		void printDivider();
		void print();
		bool stopNow();

		BeliefTreeNode& getMaxExcessUncRoot(BeliefForest& globalroot); //ADD SYLTAG

		void writeIntermediatePolicyTraceToFile(int trial, double time, const string& outFileName, string problemName);
		void progressiveIncreasePolicyInteval(int& numPolicies);
		void logFilePrint(int index);		//SYL ADDED FOR EXPTS

											// backup methods
		cacherow_stval backup(list<cacherow_stval> beliefNStates); //modified for factored, prevly: int backup(list<int> beliefs);
		cacherow_stval backupLBonly(list<cacherow_stval> beliefNStates); //modified for factored, prevly: int backup(list<int> beliefs);

																		 //second level methods
		cacherow_stval backup(cacherow_stval beliefNState); //modified for factored, prevly: int backup(int belief);
		cacherow_stval backupLBonly(cacherow_stval beliefNState); //modified for factored, prevly: int backup(int belief);

	};

}

#endif
