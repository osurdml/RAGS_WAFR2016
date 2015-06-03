// Path search class to store and search a graph
// contains functions to perform A*, Dijkstra, depth-first search, etc

#include <time.h>

class Search
{
	public:
		Search(Graph * graph, Vertex * source, Vertex * goal) ;
		~Search() ;
		
		Graph * GetGraph() const {return itsGraph ;}
		void SetGraph(Graph * graph) {itsGraph = graph ;}
		Queue * GetQueue() const {return itsQueue ;}
		void SetQueue(Queue * queue) {itsQueue = queue ;}
		Vertex * GetSource() const {return itsSource ;}
		void SetSource(Vertex * v1) {itsSource = v1 ;}
		Vertex * GetGoal() const {return itsGoal ;}
		void SetGoal(Vertex * v2) {itsGoal = v2 ;}
		
		vector<Node *> PathSearch(pathOut pType) ;
	private:
		Graph * itsGraph ;
		Queue * itsQueue ;
		Vertex * itsSource ;
		Vertex * itsGoal ;
		
		ULONG FindSourceID() ;
		double ManhattanDistance(Vertex *, Vertex *) ;
		double EuclideanDistance(Vertex *, Vertex *) ;
		void UpdateNode(Node *) ;
} ;

Search::Search(Graph * graph, Vertex* v1, Vertex * v2)
{
	itsGraph = graph ;
	itsSource = v1 ;
	itsGoal = v2 ;
}

Search::~Search()
{
	delete itsQueue ;
	itsQueue = 0 ;
}

vector<Node *> Search::PathSearch(pathOut pType)
{
	ULONG sourceID = FindSourceID() ;
	itsQueue = new Queue(new Node(itsGraph->GetVertices()[sourceID], SOURCE)) ;
	
	clock_t t_start = clock() ;
	double t_elapse = 0.0 ;
	
	while (!itsQueue->EmptyQueue() && t_elapse < 5)
	{
		// Pop cheapest node from queue
		Node * currentNode = itsQueue->PopQueue() ;
		if (!currentNode)
		{
			continue ;
		}
		
		if (pType == BEST)
		{
			// Terminate search once one path is found
			if (currentNode->GetVertex()->GetX() == itsGoal->GetX() &&
				currentNode->GetVertex()->GetY() == itsGoal->GetY())
				break ;
		}
		
		Node * currentNeighbour ;
		
		// Find all neighbours
		vector<Edge *> neighbours = itsGraph->GetNeighbours(currentNode->GetVertex()) ;

		// Update neighbours
		for (ULONG i = 0; i < (ULONG)neighbours.size(); i++)
		{
			// Check if neighbour vertex is already in closed set
			bool newNeighbour = true ;
			if (pType == BEST)
			{
				Vertex * vcheck = neighbours[i]->GetVertex2() ;
				for (ULONG j = 0; j < itsQueue->GetClosed().size(); j++)
				{
					if (itsQueue->GetClosed()[j]->GetVertex()->GetX() == vcheck->GetX() &&
						itsQueue->GetClosed()[j]->GetVertex()->GetY() == vcheck->GetY())
					{
						newNeighbour = false ;
						break ;
					}
				}
			}
			
			if (newNeighbour)
			{
				// Create neighbour node
				currentNeighbour = new Node(currentNode, neighbours[i]) ;
				UpdateNode(currentNeighbour) ;
			
				itsQueue->UpdateQueue(currentNeighbour) ;
			}
		}
		
		t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
	}
	
	// Check if a path is found
	bool ClosedAll = false ;
	for (ULONG i = 0; i < itsQueue->GetClosed().size(); i++)
	{
		if (itsQueue->GetClosed()[i]->GetVertex()->GetX() == itsGoal->GetX() &&
			itsQueue->GetClosed()[i]->GetVertex()->GetY() == itsGoal->GetY())
		{
			ClosedAll = true ;
			break ;
		}
	}
	
	if (!ClosedAll)
	{
		cout << "No path found from source to goal.\n" ;
		vector<Node *> bestPath ;
		return bestPath ;
	}
	else
	{
		ULONG k = 0 ;
		vector<Node *> bestPath((ULONG)itsQueue->GetClosed().size()) ;
		
		for (ULONG i = 0; i < (ULONG)itsQueue->GetClosed().size(); i++)
		{
			if (itsGoal->GetX() == itsQueue->GetClosed()[i]->GetVertex()->GetX() &&
				itsGoal->GetY() == itsQueue->GetClosed()[i]->GetVertex()->GetY())
			{
				bestPath[k] = itsQueue->GetClosed()[i] ;
				k++ ;
			}
		}
		
		bestPath.resize(k) ;
		//cout << "Found " << k << " best path/s from source to goal.\n" ;
		
		return bestPath ;
	}
}

ULONG Search::FindSourceID()
{
	for (ULONG i = 0; i < itsGraph->GetNumVertices(); i++)
	{
		if (itsSource->GetX() == itsGraph->GetVertices()[i]->GetX() &&
		itsSource->GetY() == itsGraph->GetVertices()[i]->GetY())
			return i ;
	}
}

double Search::ManhattanDistance(Vertex * v1, Vertex * v2)
{
	double diffX = abs(v1->GetX() - v2->GetX()) ;
	double diffY = abs(v1->GetY() - v2->GetY()) ;
	double diff = diffX + diffY ;
	return diff ;
}

double Search::EuclideanDistance(Vertex * v1, Vertex * v2)
{
	double diffX = pow(v1->GetX() - v2->GetX(),2) ;
	double diffY = pow(v1->GetY() - v2->GetY(),2) ;
	double diff = sqrt(diffX+diffY) ;
	return diff ;
}

void Search::UpdateNode(Node * n)
{
	if (SEARCH_TYPE == ASTAR)
	{
		double diff ;
		switch (HEURISTIC)
		{
			case ZERO:
				diff = 0.0 ;
				n->SetHeuristic(diff) ;
				break ;
			case MANHATTAN:
				diff = ManhattanDistance(itsGoal, n->GetVertex()) ;
				n->SetHeuristic(diff) ;
				break ;
			case EUCLIDEAN:
				diff = EuclideanDistance(itsGoal, n->GetVertex()) ;
				n->SetHeuristic(diff) ;
				break ;
		}
	}
}
