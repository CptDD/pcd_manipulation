#pragma once
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
#include "OPBeliefTreeNode.h"
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace std;
namespace momdp
{

	class treeHandler : public PointBasedAlgorithm
	{
	private:
		DHSBeliefTreeNode * solutionRootNode;
		int maxDepth, deletedNodes;
	public:
		//SolverParams * solverParams;	// inherited from PointBasedAlgorithm
		//SharedPointer<MOMDP> problem; // inherited from PointBasedAlgorithm

		treeHandler(SharedPointer<MOMDP> problem, SolverParams * solverParams);
		~treeHandler();

		void treeHandler::buildFullTree(int maxDepth, SharedPointer<MOMDP> problem, DHSBeliefTreeNode * cn);
		void treeHandler::expandPrepare(DHSBeliefTreeNode* cn, SharedPointer<MOMDP> problem);
		void treeHandler::expandOP(DHSBeliefTreeNode& cn, SharedPointer<MOMDP> problem);
		DHSBeliefTreeNode* treeHandler::getOPNode(SharedPointer<BeliefWithState>& b_s, SharedPointer<MOMDP> problem);
		DHSBeliefTreeNode* treeHandler::initOPRoot(SharedPointer<MOMDP> problem);
		DHSBeliefTreeNode * treeHandler::freeSubTree2(DHSBeliefTreeNode *cn, int A, int O, int X, bool firstLeve);

		int treeHandler::runMemFreeTest(int maxDepth, bool partialDelete);

		//virual stuff for PBA

		virtual void solve(SharedPointer<MOMDP> problem);	
		virtual void writePolicy(string fileName, string problemName);
		virtual int getOptimalAction(SharedPointer<MOMDP> problem, int action, int observation, int Xn);
		
	};

};