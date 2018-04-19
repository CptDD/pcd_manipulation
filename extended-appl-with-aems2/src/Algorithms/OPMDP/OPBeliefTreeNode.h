#ifndef OPBeliefTreeNode_H
#define OPBeliefTreeNode_H

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

	class OPBeliefTreeNode;
	//class OPBeliefCache;
	//class Tuple;

	
	class OPBeliefTreeEdge
	{
	public:
		OPBeliefTreeEdge();
		~OPBeliefTreeEdge();
		double obsProb;		// note that this is the joint prob of observation and next_x value
		OPBeliefTreeNode* nextState;
		
		
	};


	// added for factored - SYL 17072008
	class OPBeliefTreeObsState
	{
	public:
		//Tuple* extraData;
		vector<OPBeliefTreeEdge*> outcomes;
		size_t getNumOutcomes(void) const { return outcomes.size(); }
		OPBeliefTreeObsState()
		{
			///extraData = NULL;
		}
		~OPBeliefTreeObsState(void);
		
	};

	class OPBeliefTreeQEntry
	{
	public:
		double immediateReward;
		//std::vector<OPBeliefTreeEdge*> outcomes;		// removed for factored - SYL 17072008
		vector<OPBeliefTreeObsState*> stateOutcomes;	// added for factored - SYL 17072008
		//Tuple* extraData;
		double lbVal, ubVal;
		bool valid;//added rn 12/18/2006 //for purpose of record which entries are valid (not sub-optimal)
		size_t getNumStateOutcomes(void) const { return stateOutcomes.size(); }
		OPBeliefTreeQEntry(void);		
		~OPBeliefTreeQEntry(void);
	};
	
	
	class OPBeliefTreeNode : public MObject
	{
	public:
		OPBeliefTreeNode(void);
		virtual ~OPBeliefTreeNode(void);

		SharedPointer<BeliefWithState> s;				// s is composed of int s-MDP; sparse_vector b-POMDP	PAEL
		int depth;
		REAL_VALUE UB, LB;
		REAL_VALUE contribution;			// contribution of a leaf or updated contribution propagated from childs to parent	PAEL
		REAL_VALUE obsProbProduct;			// product of probabilities from parents to child	PAEL
		OPBeliefTreeNode* optimalLeafNode;	// the optimal leafe node which will be selected for next expantion phase	PAEL
		OPBeliefTreeNode* parent;			// parent of a child node	PAEL		
		int optA, optO;						// for debuging: action taken and observation chosen to get this node	PAEL
		vector<OPBeliefTreeQEntry> Q;		// MDP & POMDP edge structure to all childe nodes	PAEL

		void print();
		bool isFringe(void) const { return Q.empty(); }
		size_t getNodeNumActions(void) const { return Q.size(); }
		OPBeliefTreeNode& getNextState(int a, int o, int x) { return *Q[a].stateOutcomes[x]->outcomes[o]->nextState; }  // added for factored - SYL 17072008

	};

}


#endif
