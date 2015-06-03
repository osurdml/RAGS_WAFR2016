// Graph class to create and store graph structure
// 

class Graph
{
	public:
		Graph(vector< vector<double> > vertices, double radius) ;
		Graph(vector< vector<double> > vertices, vector< vector<double> > edges) ;
		Graph(vector<double> xGrid, vector<double> yGrid, int connect) ;
		Graph(vector< vector<bool> > mask, int connect) ;
		~Graph() ;
		
		Vertex ** GetVertices() const {return itsVertices ;}
		void SetVertices(Vertex ** vertices) {itsVertices = vertices ;}
		Edge ** GetEdges() const {return itsEdges ;}
		void SetEdges(Edge ** edges) {itsEdges = edges ;}
		ULONG GetNumVertices() const {return numVertices ;}
		void SetNumVertices(ULONG nVerts) {numVertices = nVerts ;}
		ULONG GetNumEdges() const {return numEdges ;}
		void SetNumEdges(ULONG nEdges) {numEdges = nEdges ;}
		
		vector<Edge *> GetNeighbours(Vertex *) ;
		void BasicConnect(vector<double> xGrid, vector<double> yGrid, 
			vector< vector<double> > & vertices, vector< vector<double> > & edges, int connect) ;
	private:
		Vertex ** itsVertices ;
		Edge ** itsEdges ;
		ULONG numVertices ;
		ULONG numEdges ;
		Vertex ** GenerateVertices(vector< vector<double> > vertices) ;
		Edge ** GenerateEdges(vector< vector<double> > edges) ;
		vector< vector<double> > RadiusConnect(vector< vector<double> > vertices, double radius) ;
		double EuclideanDistance(vector<double> v1, vector<double> v2) ;
} ;

Graph::Graph(vector< vector<double> > vertices, double radius) // PRM-style graph connection
{
	itsVertices = GenerateVertices(vertices) ;
	vector< vector<double> > edges = RadiusConnect(vertices, radius) ;
	itsEdges = GenerateEdges(edges) ;
	numVertices = (ULONG)vertices.size() ;
	numEdges = (ULONG)edges.size() ;
}

Graph::Graph(vector< vector<double> > vertices, vector< vector<double> > edges)
{
	itsVertices = GenerateVertices(vertices) ;
	itsEdges = GenerateEdges(edges) ;
	numVertices = (ULONG)vertices.size() ;
	numEdges = (ULONG)edges.size() ;
}

Graph::Graph(vector<double> xGrid, vector<double> yGrid, int connect)
{
	ULONG nVerts = xGrid.size()*yGrid.size() ;
	
	ULONG k = 0 ;
	vector< vector<double> > vertices(nVerts, vector<double>(2)) ;
	
	for (ULONG i = 0; i < (ULONG)xGrid.size(); i++)
	{
		for (ULONG j = 0; j < (ULONG)yGrid.size(); j++)
		{
			vertices[k][0] = xGrid[i] ;
			vertices[k][1] = yGrid[j] ;
			k++ ;
		}
	}
	numVertices = k ;
	
	// calculate number of edges
	ULONG nEdges ;
	ULONG cornerEdges ;
	ULONG xEdges ;
	ULONG yEdges ;
	ULONG otherEdges ;
	switch (connect)
	{
		case 4:
			cornerEdges = 2*4 ;
			xEdges = max(((double)xGrid.size()-2)*2 *3,0.0) ;
			yEdges = max(((double)yGrid.size()-2)*2 *3,0.0) ;
			otherEdges = max(((double)xGrid.size()-2)*((double)yGrid.size()-2) *4,0.0) ;
			nEdges = cornerEdges + xEdges + yEdges + otherEdges ;
			break ;
		case 8:
			cornerEdges = 3*4 ;
			xEdges = max(((double)xGrid.size()-2)*2 *5,0.0) ;
			yEdges = max(((double)yGrid.size()-2)*2 *5,0.0) ;
			otherEdges = max(((double)xGrid.size()-2)*((double)yGrid.size()-2) *8,0.0) ;
			nEdges = cornerEdges + xEdges + yEdges + otherEdges ;
			break ;
	}
	
	vector< vector<double> > edges(nEdges, vector<double>(4)) ;
	
	BasicConnect(xGrid, yGrid, vertices, edges, connect) ;
	itsVertices = GenerateVertices(vertices) ;
	itsEdges = GenerateEdges(edges) ;
}

Graph::Graph(vector< vector<bool> > mask, int connect)
{
	vector<double> xGrid ;
	vector<double> yGrid ;
	for (ULONG i = 0; i < mask.size(); i++)
		xGrid.push_back(i) ;
	for (ULONG i = 0; i < mask[0].size(); i++)
		yGrid.push_back(i) ;
	
	ULONG nVerts = xGrid.size()*yGrid.size() ;
	
	ULONG k = 0 ;
	vector< vector<double> > vertices(nVerts, vector<double>(2)) ;
	
	for (ULONG i = 0; i < (ULONG)xGrid.size(); i++)
	{
		for (ULONG j = 0; j < (ULONG)yGrid.size(); j++)
		{
			vertices[k][0] = xGrid[i] ;
			vertices[k][1] = yGrid[j] ;
			k++ ;
		}
	}
	numVertices = k ;
	
	// calculate number of edges
	ULONG nEdges ;
	ULONG cornerEdges ;
	ULONG xEdges ;
	ULONG yEdges ;
	ULONG otherEdges ;
	switch (connect)
	{
		case 4:
			cornerEdges = 2*4 ;
			xEdges = max(((double)xGrid.size()-2)*2 *3,0.0) ;
			yEdges = max(((double)yGrid.size()-2)*2 *3,0.0) ;
			otherEdges = max(((double)xGrid.size()-2)*((double)yGrid.size()-2) *4,0.0) ;
			nEdges = cornerEdges + xEdges + yEdges + otherEdges ;
			break ;
		case 8:
			cornerEdges = 3*4 ;
			xEdges = max(((double)xGrid.size()-2)*2 *5,0.0) ;
			yEdges = max(((double)yGrid.size()-2)*2 *5,0.0) ;
			otherEdges = max(((double)xGrid.size()-2)*((double)yGrid.size()-2) *8,0.0) ;
			nEdges = cornerEdges + xEdges + yEdges + otherEdges ;
			break ;
	}
	
	vector< vector<double> > allEdges(nEdges, vector<double>(4)) ;
	
	BasicConnect(xGrid, yGrid, vertices, allEdges, connect) ;
	
	k = 0 ;
	// Increase cost of colliding edges to max
	for (ULONG i = 0; i < mask.size(); i++)
	{
		for (ULONG j = 0; j < mask[i].size(); j++)
		{
			if (mask[i][j]) // obstacle
			{
				for (ULONG ii = 0; ii < allEdges.size(); ii++)
					if ((allEdges[ii][0] == k || allEdges[ii][1] == k) && allEdges[ii][2] < DBL_MAX)
					{
						allEdges[ii][2] = DBL_MAX ;
					}
			}
			k++ ;
		}
	}
	
	vector< vector<double> > edges ;
	ULONG ce = 0 ;
	// Select edges that are not blocked
	for (ULONG i = 0; i < allEdges.size(); i++)
	{
		if (allEdges[i][2] < DBL_MAX)
		{
			edges.push_back(allEdges[i]) ;
			ce++ ;
		}
	}
	numEdges = ce ;
	
	// Write edges and vertices to file
	ofstream edgeFile ;
	edgeFile.open("path_edges.txt") ;
	for (ULONG i = 0; i < edges.size(); i++)
	{
		for (ULONG j = 0; j < edges[i].size(); j++)
		{
			edgeFile << edges[i][j] ;
			if (j == edges[i].size()-1)
				edgeFile << "\n" ;
			else
				edgeFile << "," ;
		}
	}
	edgeFile.close() ;
	
	ofstream vertFile ;
	vertFile.open("path_vertices.txt") ;
	for (ULONG i = 0; i < vertices.size(); i++)
	{
		for (ULONG j = 0; j < vertices[i].size(); j++)
		{
			vertFile << vertices[i][j] ;
			if (j == vertices[i].size()-1)
				vertFile << "\n" ;
			else
				vertFile << "," ;
		}
	}
	vertFile.close() ;
	
	itsVertices = GenerateVertices(vertices) ;
	itsEdges = GenerateEdges(edges) ;
}

Graph::~Graph()
{
	delete [] itsVertices ;
	itsVertices = 0;
	delete [] itsEdges ;
	itsEdges = 0 ;
}

vector<Edge *> Graph::GetNeighbours(Vertex * v)
{
	vector<Edge *> neighbours(numEdges) ;
	ULONG k = 0 ;
	
	for (ULONG i = 0; i < numEdges; i++)
	{
		double x1 = itsEdges[i]->GetVertex1()->GetX() ;
		double y1 = itsEdges[i]->GetVertex1()->GetY() ;
		if (x1 == v->GetX() && y1 == v->GetY())
		{
			neighbours[k] = itsEdges[i] ;
			k++ ;
		}
	}
	
	neighbours.resize(k) ;
	return neighbours ;
}

void Graph::BasicConnect(vector<double> xGrid, vector<double> yGrid, 
	vector< vector<double> > & vertices, vector< vector<double> > & edges, int connect)
{
	ULONG p = 0 ;
	
	for (ULONG i = 0; i < (ULONG)vertices.size(); i++)
	{
		 int north = 1 ;
		 int south = 1 ;
		 int east = 1 ;
		 int west = 1 ;
		 
		 if ((i+1) % yGrid.size() == 0)
		 	north = 0 ;
		 if (i % yGrid.size() == 0)
		 	south = 0 ;
		 if (i >= (xGrid.size()-1)*(yGrid.size()))
		 	east = 0 ;
		 if (i < yGrid.size())
		 	west = 0 ;
        
		 double var = 0.0 ;
		 if (north)
		 {
		 	edges[p][0] = (double)i ;
		 	edges[p][1] = (double)(i+1) ;
		 	edges[p][2] = EuclideanDistance(vertices[i], vertices[i+1]) ;
		 	edges[p][3] = var ;
		 	p++ ;
		 	if (connect == 8 && east)
		 	{
		 		edges[p][0] = (double)i ;
		 		edges[p][1] = (double)(i+yGrid.size()+1) ;
		 		edges[p][2] = EuclideanDistance(vertices[i], vertices[i+yGrid.size()+1]) ;
		 		edges[p][3] = var ;
		 		p++ ;
		 	}
		 	if (connect == 8 && west)
		 	{
		 		edges[p][0] = (double)i ;
		 		edges[p][1] = (double)(i-yGrid.size()+1) ;
		 		edges[p][2] = EuclideanDistance(vertices[i], vertices[i-yGrid.size()+1]) ;
		 		edges[p][3] = var ;
		 		p++ ;
		 	}
		 }
		 if (south)
		 {
		 	edges[p][0] = (double)i ;
		 	edges[p][1] = (double)(i-1) ;
		 	edges[p][2] = EuclideanDistance(vertices[i], vertices[i-1]) ;
		 	edges[p][3] = var ;
		 	p++ ;
		 	if (connect == 8 && east)
		 	{
		 		edges[p][0] = (double)i ;
		 		edges[p][1] = (double)(i+yGrid.size()-1) ;
		 		edges[p][2] = EuclideanDistance(vertices[i], vertices[i+yGrid.size()-1]) ;
		 		edges[p][3] = var ;
		 		p++ ;
		 	}
		 	if (connect == 8 && west)
		 	{
		 		edges[p][0] = (double)i ;
		 		edges[p][1] = (double)(i-yGrid.size()-1) ;
		 		edges[p][2] = EuclideanDistance(vertices[i], vertices[i-yGrid.size()-1]) ;
		 		edges[p][3] = var ;
		 		p++ ;
		 	}
		 }
		 if (east)
		 {
		 	edges[p][0] = (double)i ;
		 	edges[p][1] = (double)(i+yGrid.size()) ;
		 	edges[p][2] = EuclideanDistance(vertices[i], vertices[i+yGrid.size()]) ;
		 	edges[p][3] = var ;
		 	p++ ;
		 }
		 if (west)
		 {
		 	edges[p][0] = (double)i ;
		 	edges[p][1] = (double)(i-yGrid.size()) ;
		 	edges[p][2] = EuclideanDistance(vertices[i], vertices[i-yGrid.size()]) ;
		 	edges[p][3] = var ;
		 	p++ ;
		 }
	}
	numEdges = p ;
}

Vertex ** Graph::GenerateVertices(vector< vector<double> > vertices)
{
	Vertex ** allVertices = new Vertex * [(ULONG)vertices.size()] ;
	
	for (ULONG i = 0; i < (ULONG)vertices.size(); i++)
	{
		double x = vertices[i][0] ;
		double y = vertices[i][1] ;
		
		allVertices[i] = new Vertex(x,y) ;
	}
	
	return allVertices ;
}

Edge ** Graph::GenerateEdges(vector< vector<double> > edges)
{
	Edge ** allEdges = new Edge * [(ULONG)edges.size()] ;
	
	for (ULONG i = 0; i < (ULONG)edges.size(); i++)
	{
		Vertex * v1 = itsVertices[(ULONG)edges[i][0]] ;
		Vertex * v2 = itsVertices[(ULONG)edges[i][1]] ;
		double cost = edges[i][2] ;
		double var = edges[i][3] ;
		
		allEdges[i] = new Edge(v1, v2, cost, var) ;
	}
	
	return allEdges ;
}

// Connect vertices within specified radius
vector< vector<double> > Graph::RadiusConnect(vector< vector<double> > vertices, double radius)
{
	srand(time(NULL));
	vector< vector<double> > edges(pow(vertices.size(),2), vector<double>(4)) ;
	ULONG k = 0 ;
	
	for (ULONG i = 0; i < (ULONG)vertices.size(); i++)
	{
		for (ULONG j = 0; j < (ULONG)vertices.size(); j++)
		{
			double diff = EuclideanDistance(vertices[i], vertices[j]) ;
			
			if (diff <= radius && i != j)
			{
				edges[k][0] = (double)i ;
				edges[k][1] = (double)j ;
				edges[k][2] = diff + (double)(rand() % 10000)/100.0 ;
				edges[k][3] = (double)(rand() % 2000)/100.0 ; // 0.0 ;
				k++ ;
			}
		}
	}
	
	edges.resize(k) ;
	
	// Write edges to txt file
	stringstream eFileName ;
	eFileName << "config_files/edges" << trialNum << ".txt" ;
	
	ofstream edgesFile ;
	edgesFile.open(eFileName.str().c_str()) ;
	
	for (ULONG i = 0; i < edges.size(); i++)
	{
		edgesFile << edges[i][0] << "," << edges[i][1] << ","
			<< edges[i][2] << "," << edges[i][3] << "\n" ;
	}
	edgesFile.close() ;
	
	return edges ;
}

double Graph::EuclideanDistance(vector<double> v1, vector<double> v2)
{
	double diff_x = v1[0] - v2[0] ;
	double diff_y = v1[1] - v2[1] ;
	double diff = sqrt(pow(diff_x,2) + pow(diff_y,2)) ;
	
	return diff ;
}
