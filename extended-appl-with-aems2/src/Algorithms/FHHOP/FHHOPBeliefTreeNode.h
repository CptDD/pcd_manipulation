#ifndef FHHOPBeliefTreeNode_H
#define FHHOPBeliefTreeNode_H

#include <vector>
#include "Const.h"
#include "FacmodelStructs.h"
#include "MObject.h"


#ifndef NULL
#define NULL   ((void *) 0)
#endif


using namespace std;
using namespace momdp;
namespace momdp
{
#define CB_QVAL_UNDEFINED (-99e+20)
#define CB_INITIALIZATION_PRECISION_FACTOR (1e-2)

	class FHHOPBeliefTreeNode;
	//class OPBeliefCache;
	//class Tuple;

	
	class FHHOPBeliefTreeEdge
	{
	public:
		FHHOPBeliefTreeEdge();
		~FHHOPBeliefTreeEdge();
		double obsProb;		// note that this is the joint prob of observation and next_x value
		FHHOPBeliefTreeNode* nextState;
	};


	// added for factored - SYL 17072008
	class FHHOPBeliefTreeObsState
	{
	public:
		//Tuple* extraData;
		vector<FHHOPBeliefTreeEdge*> outcomes;
		size_t getNumOutcomes(void) const { return outcomes.size(); }
		FHHOPBeliefTreeObsState()
		{
			///extraData = NULL;
		}
		~FHHOPBeliefTreeObsState(void);

	};

	class FHHOPBeliefTreeQEntry
	{
	public:
		double immediateReward;
		//std::vector<OPBeliefTreeEdge*> outcomes;		// removed for factored - SYL 17072008
		vector<FHHOPBeliefTreeObsState*> stateOutcomes;	// added for factored - SYL 17072008
		//Tuple* extraData;
		double lbVal, ubVal;
		bool valid;//added rn 12/18/2006 //for purpose of record which entries are valid (not sub-optimal)
		size_t getNumStateOutcomes(void) const { return stateOutcomes.size(); }
		FHHOPBeliefTreeQEntry(void);
		~FHHOPBeliefTreeQEntry(void);
	};
	
	
	class FHHOPBeliefTreeNode : public MObject
	{
	public:
		FHHOPBeliefTreeNode(void);
		virtual ~FHHOPBeliefTreeNode(void);

		SharedPointer<BeliefWithState> s;				// s is composed of int s-MDP; sparse_vector b-POMDP	PAEL
		int depth;
		REAL_VALUE UB, LB, deltaUB, deltaLB;
		REAL_VALUE contribution_AEMS2;			// contribution of a leaf or updated contribution propagated from childs to parent	PAEL
		REAL_VALUE contribution_FHHOP;			// contribution of a leaf or updated contribution propagated from childs to parent	PAEL
		REAL_VALUE obsProbProduct;			// product of probabilities from parents to child	PAEL
		FHHOPBeliefTreeNode* optimalLeafNode_AEMS2;	// the optimal leafe node which will be selected for next expantion phase	PAEL
		FHHOPBeliefTreeNode* optimalLeafNode_FHHOP;	// the optimal leafe node which will be selected for next expantion phase	PAEL
		FHHOPBeliefTreeNode* parent;			// parent of a child node	PAEL
		REAL_VALUE parentContribution_AEMS2;
		REAL_VALUE parentContribution_FHHOP;

		int optA, optO;						// for debuging: action taken and observation chosen to get this node	PAEL
		vector<FHHOPBeliefTreeQEntry> Q;		// MDP & POMDP edge structure to all childe nodes	PAEL

		void print();
		bool isFringe(void) const { return Q.empty(); }
		size_t getNodeNumActions(void) const { return Q.size(); }
		FHHOPBeliefTreeNode& getNextState(int a, int o, int x) { return *Q[a].stateOutcomes[x]->outcomes[o]->nextState; }  // added for factored - SYL 17072008

	};

}


#endif
