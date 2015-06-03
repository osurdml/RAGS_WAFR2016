// Node class to maintain path information up to a vertex
// contains parent node, (mu, sigma) of cost-to-come

enum nodeType {SOURCE, OTHER} ;

class Node
{
	public:
		Node(Vertex *) ;
		Node(Vertex *, nodeType) ;
		Node(Node *, Edge *) ;
		~Node() ;
		
		Node * GetParent() const {return itsParent ;}
		void SetParent(Node * parent) {itsParent = parent ;}
		double GetMeanCost() const {return itsMeanCost ;}
		void SetMeanCost(double cost) {itsMeanCost = cost ;}
		double GetVarCost() const {return itsVarCost ;}
		void SetVarCost(double var) {itsVarCost = var ;}
		Vertex * GetVertex() const {return itsVertex ;}
		void SetVertex(Vertex * vertex) {itsVertex = vertex ;}
		ULONG GetDepth() const {return itsDepth ;}
		void SetDepth(ULONG depth) {itsDepth = depth ;}
		double GetHeuristic() const {return itsHeuristic ;}
		void SetHeuristic(double h) {itsHeuristic = h ;}
		double GetMeanCTG() const {return itsMeanCTG ;}
		double GetVarCTG() const {return itsVarCTG ;}
		
		void DisplayPath() ;
		Node * ReverseList(Node * itsChild) ;
		void SetCTG(double totalMean, double totalVar) ;
	private:
		Vertex * itsVertex ;
		Node * itsParent ;
		double itsMeanCost ;
		double itsVarCost ;
		ULONG itsDepth ;
		double itsHeuristic ;
		double itsMeanCTG ;
		double itsVarCTG ;
} ;

Node::Node(Vertex * vertex)
{
	itsVertex = vertex ;
	itsParent = 0 ;
	itsMeanCost = 0.0 ;
	itsVarCost = 0.0 ;
	itsDepth = 0 ;
	itsHeuristic = 0.0 ;
	itsMeanCTG = 0.0 ;
	itsVarCTG = 0.0 ;
}

Node::Node(Vertex * vertex, nodeType n)
{
	switch (n)
	{
		case SOURCE:
			itsVertex = vertex ;
			itsParent = 0 ;
			itsMeanCost = 0.0 ;
			itsVarCost = 0.0 ;
			itsDepth = 0.0 ;
			itsHeuristic = 0.0 ;
			itsMeanCTG = 0.0 ;
			break ;
		default:
			itsVertex = vertex ;
			itsParent = 0 ;
			itsMeanCost = DBL_MAX ;
			itsVarCost = DBL_MAX ;
			itsDepth = ULONG_MAX ;
			itsHeuristic = 0.0 ;
			itsMeanCTG = DBL_MAX ;
			itsVarCTG = DBL_MAX ;
	}
}

Node::Node(Node * parent, Edge * edge)
{
	itsVertex = edge->GetVertex2() ;
	itsParent = parent ;
	itsMeanCost = itsParent->GetMeanCost() + edge->GetMeanSearch() ;
	itsVarCost = itsParent->GetVarCost() + edge->GetVarSearch() ;
	itsDepth = itsParent->GetDepth() + 1 ;
	itsHeuristic = 0.0 ;
	itsMeanCTG = 0.0 ;
	itsVarCTG = 0.0 ;
}

Node::~Node()
{
}

void Node::DisplayPath()
{
	cout << "Vertex: (" << itsVertex->GetX() << "," << itsVertex->GetY() << ")\n" ;
	cout << "Mean cost-to-come: " << itsMeanCost << ", " << "variance: " << itsVarCost << endl ;
	cout << "Mean cost-to-go: " << itsMeanCTG << ", " << "variance: " << itsVarCTG << endl ;
	
	if (itsParent)
		itsParent->DisplayPath() ;
}

Node * Node::ReverseList(Node * itsChild)
{
	Node * itsParentR = new Node(GetVertex()) ;
	itsParentR->SetMeanCost(GetMeanCost()) ;
	itsParentR->SetVarCost(GetVarCost()) ;
	itsParentR->SetParent(itsChild) ;
	
	Node * itsReverse ;
	
	if (GetParent())
	{
		itsReverse = GetParent()->ReverseList(itsParentR) ;
		return itsReverse ;
	}
	else
		return itsParentR ;
	
}

void Node::SetCTG(double totalMean, double totalVar)
{
	itsMeanCTG = totalMean - itsMeanCost ;
	itsVarCTG = totalVar - itsVarCost ;
	
	//cout << "Mean CTG :" << itsMeanCTG << ", var CTG: " << itsVarCTG << endl ;
	
	if (itsParent)
		itsParent->SetCTG(totalMean, totalVar) ;
}
