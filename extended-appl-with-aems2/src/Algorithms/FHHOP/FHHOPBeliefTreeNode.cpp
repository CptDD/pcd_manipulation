//
//	Algorithms/OPMDP/OPBeleifTreeNode.cpp
//
//	Copyright (c) 2008-2010 by National University of Singapore.
//	Modifications Copyright (c) 2015-2016 by Technical University of Cluj-Napoca.
//
//
//	Under GNU General Public License V2.
//

#include "FHHOPBeliefTreeNode.h"



FHHOPBeliefTreeNode::FHHOPBeliefTreeNode(void)
{
	s = NULL;		
}



FHHOPBeliefTreeNode::~FHHOPBeliefTreeNode(void)
{

	this->optimalLeafNode_FHHOP = NULL;
	this->optimalLeafNode_AEMS2 = NULL;
	this->parent = NULL;
	this->s = NULL;// .~intrusive_ptr();
	this->Q.clear();
	/*
	FOR(i, this->getNodeNumActions())
	{
		this->Q[i].~OPBeliefTreeQEntry();
	}
	*/
	//vector<OPBeliefTreeQEntry>().swap(this->Q);

}

void FHHOPBeliefTreeNode::print()
{
	cout << "BeliefTreeNode:" << endl;
	//cout << "CacheIndex row sval " << cacheIndex.row << " " << cacheIndex.sval << endl;
	cout << "s beleif ";
	s->bvec->write(cout) << endl;
	cout << " sval " << s->sval << endl;
	
	for(vector<FHHOPBeliefTreeQEntry>::iterator iter1 = Q.begin() ; iter1 != Q.end() ; iter1 ++)
	{
		FHHOPBeliefTreeQEntry&  entry= *iter1;
		for(vector<FHHOPBeliefTreeObsState*>::iterator iter2 = entry.stateOutcomes.begin() ; iter2 != entry.stateOutcomes.end() ; iter2 ++)
		{
			FHHOPBeliefTreeObsState* obsState = *iter2;
			if(!obsState)
			{
				continue;
			}

			for(vector<FHHOPBeliefTreeEdge*>::iterator iter3 = obsState->outcomes.begin() ; iter3 != obsState->outcomes.end() ; iter3 ++)
			{
				FHHOPBeliefTreeEdge* edge = *iter3;
				if(edge)
				{
					edge->nextState->print();
				}
			}

		}
	}
}



FHHOPBeliefTreeEdge::FHHOPBeliefTreeEdge()
{
	this->nextState = NULL;
	this->obsProb = -1;
}

FHHOPBeliefTreeEdge::~FHHOPBeliefTreeEdge()
{
	//if (this->nextState != NULL)
	if (this->nextState)
	{
		//cout << "ptr not null at addr: " /*<< this->nextState */ << endl;
		delete this->nextState;// ->~OPBeliefTreeNode();
	}
}

FHHOPBeliefTreeQEntry::FHHOPBeliefTreeQEntry()
{
	// to be added if needed	PAEL
}

FHHOPBeliefTreeQEntry::~FHHOPBeliefTreeQEntry()
{
	//this->stateOutcomes.clear();

	FOR(Xni, this->getNumStateOutcomes())
	{
		if (this->stateOutcomes[Xni])
		{
			//this->stateOutcomes[Xni]->~OPBeliefTreeObsState();
			delete this->stateOutcomes[Xni];
		}
	}


	//vector<OPBeliefTreeObsState*>().swap(this->stateOutcomes);
}


FHHOPBeliefTreeObsState::~FHHOPBeliefTreeObsState(void)
{
	//this->outcomes.clear();

	FOR(Oval, this->getNumOutcomes())
	{

		if (this->outcomes[Oval])
		{
			delete this->outcomes[Oval]; // ->~OPBeliefTreeEdge();
		}
	}

	//vector<OPBeliefTreeEdge*>().swap(this->outcomes);

}
