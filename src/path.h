// Handles low level path planning through given sector ordering

class Path
{
	public:
		Path(vector< vector<bool> > obstacles, vector< vector<int> > membership, Node * sectorOrder,
		Vertex * source, Vertex * goal) ;
		~Path() ;
		
		vector< vector<bool> > GetObstacles() const {return itsObstacles ;}
		void SetObstacles(vector< vector<bool> > obstacles) {itsObstacles = obstacles ;}
		vector< vector<int> > GetMembership() const {return itsMembership ;}
		void SetMembership(vector< vector<int> > membership) {itsMembership = membership ;}
		Node * GetSectorOrder() const {return itsSectorOrder ;}
		void SetSectorOrder(Node * sectorOrder) {itsSectorOrder = sectorOrder ;}
		Vertex * GetSource() const {return itsSource ;}
		void SetSource(Vertex * source) {itsSource = source ;}
		Vertex * GetGoal() const {return itsGoal ;}
		void SetGoal(Vertex * goal) {itsGoal = goal ;}
		
		vector< vector<bool> > GetMask() const {return itsMask ;}
		Graph * GetGraph() const {return itsGraph ;}
		
		Node * ComputePath() ;
		Node * ComputePath(int connect) ;
	private:
		vector< vector <bool> > itsObstacles ;
		vector< vector<int> > itsMembership ;
		Node * itsSectorOrder ;
		Vertex * itsSource ;
		Vertex * itsGoal ;
		vector< vector <bool> > itsMask ;
		Graph * itsGraph ;
		Search * itsSearch ;
		Node * itsPath ;
		int itsConnect ;
		
		void CreateMask() ;
} ;

Path::Path(vector< vector<bool> > obstacles, vector< vector<int> > membership, Node * sectorOrder,
		Vertex * source, Vertex * goal)
{
	itsObstacles = obstacles ;
	itsMembership = membership ;
	itsSectorOrder = sectorOrder ;
	itsSource = source ;
	itsGoal = goal ;
	itsConnect = 8 ; // default 8 connected grid
	
	CreateMask() ;
}

Path::~Path()
{
	delete itsGraph ;
	itsGraph = 0 ;
	delete itsSearch ;
	itsSearch = 0 ;
	delete itsPath ;
	itsPath = 0 ;
}

Node * Path::ComputePath()
{
	// Create graph object
	cout << "Creating low level graph..." ;
	Graph * itsGraph = new Graph(itsMask, itsConnect) ;
	cout << "complete.\n" ;
	
	// Create search object
	cout << "Creating search object..." ;
	Search * itsSearch = new Search(itsGraph, itsSource, itsGoal) ;
	cout << "complete.\n" ;
	
	cout << "Searching for best path...\n" ;
	pathOut pType = BEST ;
	vector<Node *> bestPaths = itsSearch->PathSearch(pType) ;
	itsPath = bestPaths[0] ;
	
	// Write path to txt file
	Node * currentPathNode = itsPath ;
	ofstream pathFile ;
	pathFile.open("path.txt") ;
	while (currentPathNode)
	{
		pathFile << currentPathNode->GetVertex()->GetX() << "," 
			<< currentPathNode->GetVertex()->GetY() << "\n" ;
		currentPathNode = currentPathNode->GetParent() ;
	}
	pathFile.close() ;
	
	return itsPath ;
}

Node * Path::ComputePath(int connect)
{
	itsConnect = connect ;
	itsPath = ComputePath() ;
	return itsPath ;
}

void Path::CreateMask()
{
	itsMask = itsObstacles ;
	vector<int> sectorMembership ; // store valid sector numbers
	
	Node * currentSector = itsSectorOrder ;
	
	while (currentSector)
	{
		sectorMembership.push_back(itsMembership[currentSector->GetVertex()->GetX()]
			[currentSector->GetVertex()->GetY()]) ;
		currentSector = currentSector->GetParent() ;
	}
	
	for (int i = 0; i < itsMask.size(); i++)
	{
		for (int j = 0; j < itsMask[i].size(); j++)
		{
			bool valid = false ;
			for (int k = 0; k < sectorMembership.size(); k++)
			{
				if (itsMembership[i][j] == sectorMembership[k])
				{
					valid = true ;
					break ;
				}
			}
			
			if (!valid)
				itsMask[i][j] = true ; // should not pass through this cell (not on sector route)
		}
	}
	
	// Write mask to txt file
	ofstream maskFile ;
	maskFile.open("mask.txt") ;
	for (int i = 0; i < itsMask.size(); i++)
	{
		for (int j = 0; j < itsMask[i].size(); j++)
		{
			maskFile << itsMask[i][j] ;
			if (j == itsMask[i].size()-1)
				maskFile << "\n" ;
			else
				maskFile << "," ;
		}
	}
	maskFile.close() ;
}
