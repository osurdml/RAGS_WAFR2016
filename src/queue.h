// Custom queue type to perform priority queue updates
#include "comparenode.h"

typedef priority_queue<Node *, vector<Node *>, CompareNode> QUEUE ;

class Queue
{
	public:
		Queue(Node * source) ;
		~Queue() ;
		
		vector<Node *> GetClosed() const {return closed ;}
		
		bool EmptyQueue() const {return itsPQ->empty() ;} ;
		ULONG SizeQueue() const {return (ULONG)itsPQ->size() ;}
		void UpdateQueue(Node * newNode) ;
		Node * PopQueue() ;
	private:
		QUEUE * itsPQ ;
		vector<Node *> closed ;
		bool CompareNodes(const Node * n1, const Node * n2) const ;
} ;

Queue::Queue(Node * source)
{
	itsPQ = new QUEUE ;
	itsPQ->push(source) ;
}

Queue::~Queue()
{
	delete itsPQ ;
	itsPQ = 0 ;
	for (ULONG i = 0; i < closed.size(); i ++)
	{
		delete closed[i] ;
		closed[i] = 0 ;
	}
}

void Queue::UpdateQueue(Node * newNode)
{
	// Compare newNode to nodes in closed set
	// if closed contains node with same vertex, compare their costs
	// choose whether or not to create a new node
	bool dom = false ;
	for (ULONG i = 0; i < closed.size(); i++)
	{
		if (closed[i]->GetVertex()->GetX() == newNode->GetVertex()->GetX() &&
			closed[i]->GetVertex()->GetY() == newNode->GetVertex()->GetY())
		{
			dom = CompareNodes(newNode, closed[i]) ;
			if (dom)
				return ;
		}
	}
	itsPQ->push(newNode) ;
}

Node * Queue::PopQueue()
{
	// Check if next node is already dominated by existing node in closed set
	Node * newNode = itsPQ->top() ;
	bool dom = false ;
	for (ULONG i = 0; i < closed.size(); i++)
	{
		if (closed[i]->GetVertex()->GetX() == newNode->GetVertex()->GetX() &&
			closed[i]->GetVertex()->GetY() == newNode->GetVertex()->GetY())
		{
			dom = CompareNodes(newNode, closed[i]) ;
			if (dom)
			{
				itsPQ->pop() ;
				return 0 ;
			}
		}
	}
	closed.push_back(itsPQ->top()) ;
	itsPQ->pop() ;
	return closed[(ULONG)closed.size()-1] ;
}

bool Queue::CompareNodes(const Node * n1, const Node * n2) const
{
	double n1Cost = n1->GetMeanCost() ;
	double n2Cost = n2->GetMeanCost() ;
	return (n1Cost >= n2Cost && n1->GetVarCost() >= n2->GetVarCost()) ;
/*	if (n1Cost > n2Cost && n1->GetVarCost() > n2->GetVarCost())
		return true ;
	else if (n2Cost > n1Cost && n2->GetVarCost() > n1->GetVarCost())
		return false ;
	else
		return (n1Cost > n2Cost) ;*/
}
