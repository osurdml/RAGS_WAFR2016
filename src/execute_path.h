#include <vector>
#include <math.h>
#include <limits.h>

const double pi = 3.14159265358979323846264338328 ;

vector<double> linspace(double a, double b, int n)
{
	vector<double> array ;
	double step = (b-a)/(n-1) ;
	while (a<=b)
	{
		array.push_back(a) ;
		a += step ;
	}
	return array ;
}

bool ComputeImprovementProbability(Vertex * A, Vertex * B)
{
	vector<Node *> ANodes = A->GetNodes();
	vector<Node *> BNodes = B->GetNodes();
	
	double c_A0 = A->GetCTC() ;
	double c_B0 = B->GetCTC() ;
	double max_3sig = ANodes[0]->GetMeanCTG() + 3*ANodes[0]->GetVarCTG() ;
	double min_3sig = ANodes[0]->GetMeanCTG() - 3*ANodes[0]->GetVarCTG() ;
	for (int i = 0; i < ANodes.size(); i++)
	{
		if (max_3sig < ANodes[i]->GetMeanCTG()+3*ANodes[i]->GetVarCTG())
			max_3sig = ANodes[i]->GetMeanCTG()+3*ANodes[i]->GetVarCTG() ;
		if (min_3sig > ANodes[i]->GetMeanCTG()-3*ANodes[i]->GetVarCTG())
			min_3sig = ANodes[i]->GetMeanCTG()-3*ANodes[i]->GetVarCTG() ;
	}
	for (int i = 0; i < BNodes.size(); i++)
	{
		if (max_3sig < BNodes[i]->GetMeanCTG()+3*BNodes[i]->GetVarCTG())
			max_3sig = BNodes[i]->GetMeanCTG()+3*BNodes[i]->GetVarCTG() ;
		if (min_3sig > BNodes[i]->GetMeanCTG()-3*BNodes[i]->GetVarCTG())
			min_3sig = BNodes[i]->GetMeanCTG()-3*BNodes[i]->GetVarCTG() ;
	}
	
	int n = 10000 ;
	vector<double> x = linspace(min_3sig,max_3sig,n) ;
	double dx = x[1]-x[0] ;
	double pImprove = 0.0 ;
	for (int k = 0; k < x.size(); k++)
	{
		double p_cAi = 0.0 ;
		for (int i = 0; i < ANodes.size(); i++)
		{
			double mu_Ai = ANodes[i]->GetMeanCTG() ;
			double sig_Ai = ANodes[i]->GetVarCTG() ;
			double p_cA1 = (1/(sig_Ai*sqrt(2*pi)))*exp(-(pow(x[k]-mu_Ai,2))/(2*pow(sig_Ai,2))) ;
			double p_cA2 = 1.0 ;
			for (int j = 0; j < ANodes.size(); j++)
			{
				double mu_Aj = ANodes[j]->GetMeanCTG() ;
				double sig_Aj = ANodes[j]->GetVarCTG() ;
				if (j != i)
					p_cA2 *= 0.5*erfc((x[k]-mu_Aj)/(sig_Aj*sqrt(2))) ;
			}
			p_cAi += p_cA1*p_cA2 ;
		}
		double p_cBi = 1.0 ;
		for (int i = 0; i < BNodes.size(); i++)
		{
			double mu_Bi = BNodes[i]->GetMeanCTG() ;
			double sig_Bi = BNodes[i]->GetVarCTG() ;
			p_cBi *= 0.5*erfc((x[k]-(c_B0-c_A0)-mu_Bi)/(sig_Bi*sqrt(2))) ;
		}
		pImprove += (p_cAi)*(1-p_cBi)*dx ;
	}
	
	return (pImprove<=0.5);
}

bool GreedyComparison(Vertex* A, Vertex* B)
{
	return (A->GetCTC() < B->GetCTC()) ;
}

void AssignTrueEdgeCosts(Graph * searchGraph, int seed)
{
	Edge ** allEdges = searchGraph->GetEdges() ;
	ULONG numEdges = searchGraph->GetNumEdges() ;
	default_random_engine generator(seed) ;
	cout << "Using generator(" << generator() << ")...\n" ;
	for (ULONG i = 0; i < numEdges; i++)
		allEdges[i]->SetTrueCost(generator) ;
	
	/*// Write true costs to txt file
	ofstream edgesFile ;
	edgesFile.open("config_files/true_edges.txt") ;
	for (ULONG i = 0; i < numEdges; i++)
	{
		edgesFile << allEdges[i]->GetTrueCost() << endl ;
	}
	edgesFile.close() ;*/
}

void SetTrueEdgeCosts(Vertex * v1, vector<Vertex *> v2, Graph * searchGraph)
{
	// Find all outgoing edges
	Edge ** allEdges = searchGraph->GetEdges() ;
	ULONG numEdges = searchGraph->GetNumEdges() ;
	vector<Edge *> connectedEdges ;
	for (int j = 0; j < numEdges; j++)
	{
		if (allEdges[j]->GetVertex1()->GetX() == v1->GetX() && 
			allEdges[j]->GetVertex1()->GetY() == v1->GetY())
			connectedEdges.push_back(allEdges[j]) ;
	}
	
	// Set cost-to-come of v2 vertices as true cost of edge traversal
	for (int i = 0; i < v2.size(); i++)
	{
		for (int j = 0; j < connectedEdges.size(); j++)
		{
			if (connectedEdges[j]->GetVertex2()->GetX() == v2[i]->GetX() && 
				connectedEdges[j]->GetVertex2()->GetY() == v2[i]->GetY())
			{
				v2[i]->SetCTC(connectedEdges[j]->GetTrueCost()) ;
				break ;
			}
		}
	}
}

void ResetEdges(Graph * graph)
{
	Edge ** graphEdges = graph->GetEdges() ;
	ULONG numEdges = graph->GetNumEdges() ;
	for (ULONG i = 0; i < numEdges; i++)
	{
		graphEdges[i]->SetMeanSearch(graphEdges[i]->GetMeanCost()) ;
		graphEdges[i]->SetVarSearch(graphEdges[i]->GetVarCost()) ;
	}
}

double DynamicAStar(Graph * graph, Vertex * source, Vertex * goal)
{
	double cost = 0.0 ;
	Vertex * curLoc = source ;
	
	clock_t t_start = clock() ;
	double t_elapse = 0.0 ;
	
	while ((curLoc->GetX() != goal->GetX() || curLoc->GetY() != goal->GetY()) && t_elapse < 5)
	{
		cout << "Current Location: (" << curLoc->GetX() << "," <<  curLoc->GetY() << ")\n" ;
		
		// Assign true costs to edges from current vertex
		vector<Edge *> curEdges = graph->GetNeighbours(curLoc) ;
		for (int i = 0; i < curEdges.size(); i++)
		{
			curEdges[i]->SetMeanSearch(curEdges[i]->GetTrueCost()) ;
			curEdges[i]->SetVarSearch(0.0) ;
		}
		
		//Create search object and perform path search from source to goal
		Search * testSearch = new Search(graph, curLoc, goal) ;
		pathOut pType = BEST ;
		vector<Node *> bestPathsGS = testSearch->PathSearch(pType) ;
		
		// Reverse path
		Node * bestPathSG = bestPathsGS[0]->ReverseList(0) ;
		
		// Step through path and accumulate traversal cost
		curLoc = bestPathSG->GetParent()->GetVertex() ;
		cost += bestPathSG->GetParent()->GetMeanCost() ;
		
		// Delete pointers on the heap
		delete testSearch ;
		testSearch = 0 ;
		
		t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
	}
	
	if (curLoc->GetX() != goal->GetX() || curLoc->GetY() != goal->GetY())
	{
		cout << "Failed to reach goal vertex!\n" ;
		cost = DBL_MAX ;
	}
	else
		cout << "Goal vertex (" << curLoc->GetX() << "," <<  curLoc->GetY() << ") reached! "
			<< "Total cost: " << cost << endl ;
		
	// Reset edge search costs
	ResetEdges(graph) ;
	
	return cost ;
}

double OptimalAStar(Graph * graph, Vertex * source, Vertex * goal)
{
	// Search using true costs of all edges
	ULONG numEdges = graph->GetNumEdges() ;
	for (ULONG i = 0; i < numEdges; i++)
	{
		graph->GetEdges()[i]->SetMeanSearch(graph->GetEdges()[i]->GetTrueCost()) ;
		graph->GetEdges()[i]->SetVarSearch(0.0) ;
	}
	
	//Create search object and perform path search from source to goal
	Search * testSearch = new Search(graph, source, goal) ;
	pathOut pType = BEST ;
	vector<Node *> bestPathsGS = testSearch->PathSearch(pType) ;
	
	double cost = 0.0 ;
	
	if (bestPathsGS.size() != 0)
	{
		cost = bestPathsGS[0]->GetMeanCost() ;
		cout << "Total cost: " << cost << endl ;
	}
	else
	{
		cout << "Failed to reach goal vertex!\n" ;
		cost = DBL_MAX ;
	}
	
	// Reset edge search costs
	ResetEdges(graph) ;
	
	// Delete pointers on the heap
	delete testSearch ;
	testSearch = 0 ;
	
	return cost ;
}

vector<double> executePath(vector< Node*> GSPaths, Graph * searchGraph)
{
	vector <Node *> SGPaths ; // store all paths, start to goal
	vector <Node *> newNodes, tmpNodes ; // store remaining paths, start to goal
	vector <Vertex *> nextVerts ; // store connected vertices
	vector<double> allCosts ; // store costs of PGsearch, A*, D* searches
	double totalCost = 0 ; // store total cost
	
	// Set linked list from start node to goal node
	for(int i = 0; i < GSPaths.size(); i++)
		SGPaths.push_back(GSPaths[i]->ReverseList(0));
	
	// Compute cost-to-go for all path nodes
	for(int i = 0; i < SGPaths.size(); i++)
		SGPaths[i]->SetCTG(GSPaths[i]->GetMeanCost(),GSPaths[i]->GetVarCost()) ;
	
	/**********************************************************************************************/
	// Step through paths
	cout << "Traversing PG search paths..." << endl ;
	totalCost = 0 ;
	Vertex * curLoc = SGPaths[0]->GetVertex() ;
	Vertex * goal = GSPaths[0]->GetVertex() ;
	newNodes = SGPaths ;

	// Assign nodes to first vertex
	curLoc->SetNodes(newNodes) ;
	
	while (curLoc->GetX() != goal->GetX() || curLoc->GetY() != goal->GetY())
	{
		cout << "Current Location: (" << curLoc->GetX() << "," <<  curLoc->GetY() << ")\n" ;
		
		// Extract nodes of current vertex
		newNodes = curLoc->GetNodes() ;
		
		// Display all nodes
		/*for (int i = 0; i < newNodes.size(); i++)
		{
			cout << "Node " << i << ": (" << newNodes[i]->GetVertex()->GetX() << ","
			<< newNodes[i]->GetVertex()->GetY() << ") (" << newNodes[i]->GetParent()->GetVertex()->GetX()
			<< "," << newNodes[i]->GetParent()->GetVertex()->GetY() << ")\n" ;
		}*/
		
		// Identify next vertices
		for (int i = 0; i < newNodes.size(); i++)
		{
			bool newVert = true ;
			for (int j = 0; j < nextVerts.size(); j++)
			{
				if ((nextVerts[j]->GetX() == newNodes[i]->GetParent()->GetVertex()->GetX() &&
					nextVerts[j]->GetY() == newNodes[i]->GetParent()->GetVertex()->GetY()) ||
					(nextVerts[j]->GetX() == curLoc->GetX() && nextVerts[j]->GetY() == curLoc->GetY()))
				{
					newVert = false ;
					break ;
				}
			}
			if (newVert)
				nextVerts.push_back(newNodes[i]->GetParent()->GetVertex()) ;
		}
		
		// Display next vertices
		/*cout << "Vertices: \n" ;
		for (int i = 0; i < nextVerts.size(); i++)
		{
			cout << "(" << nextVerts[i]->GetX() << "," << nextVerts[i]->GetY() << ")\n" ;
		}*/
		
		// Identify next vertex path nodes
		for (int i = 0; i < nextVerts.size(); i++)
		{
			tmpNodes.clear() ;
			for (int j = 0; j < newNodes.size(); j++)
			{
				if (nextVerts[i]->GetX() == newNodes[j]->GetParent()->GetVertex()->GetX() &&
					nextVerts[i]->GetY() == newNodes[j]->GetParent()->GetVertex()->GetY())
					tmpNodes.push_back(newNodes[j]->GetParent()) ;
			}
			nextVerts[i]->SetNodes(tmpNodes) ;
		}
		
		// Set cost-to-come for next vertices
		SetTrueEdgeCosts(curLoc, nextVerts, searchGraph) ;
		
		// Rank next vertices according to probability of improvement
		sort(nextVerts.begin(),nextVerts.end(),ComputeImprovementProbability) ;
		
		// Move to best vertex and log the cost
		curLoc = nextVerts[0] ;
		totalCost += nextVerts[0]->GetCTC() ;
		
		// Clear vectors for next step
		newNodes.clear() ;
		nextVerts.clear() ;
	}
	
	cout << "Goal vertex (" << curLoc->GetX() << "," <<  curLoc->GetY() << ") reached! "
		<< "Total cost: " << totalCost << endl ;
	allCosts.push_back(totalCost) ;
	
	newNodes.clear() ;
	nextVerts.clear() ;
	
	/**********************************************************************************************/
	// Traverse A* path (lowest mean cost path, no adaptivity)
	double lowestCost = GSPaths[0]->GetMeanCost() ;
	double lowestCostInd = 0 ;
	for (int i = 0; i < GSPaths.size(); i++)
	{
		if (GSPaths[i]->GetMeanCost() < lowestCost)
		{
			lowestCost = GSPaths[i]->GetMeanCost() ;
			lowestCostInd = i ;
		}
	}
	
	cout << "Traversing A* path...\n" ;
	totalCost = 0 ; // reset path cost
	Node * curNode = SGPaths[lowestCostInd] ;
	curLoc = curNode->GetVertex() ;
	while (curNode->GetParent())
	{
		// Identify edge to traverse
		nextVerts.push_back(curNode->GetParent()->GetVertex()) ;
		
		cout << "Current Location: (" << curLoc->GetX() << "," <<  curLoc->GetY() << ")\n" ;
		
		// Set cost-to-come for next vertex
		SetTrueEdgeCosts(curLoc, nextVerts, searchGraph) ;
		
		// Move to next vertex and log the cost
		curNode = curNode->GetParent() ;
		curLoc = curNode->GetVertex() ;
		totalCost += nextVerts[0]->GetCTC() ;
		
		// Clear vectors for next step
		newNodes.clear() ;
		nextVerts.clear() ;
	}
	
	cout << "Goal vertex (" << curLoc->GetX() << "," <<  curLoc->GetY() << ") reached! "
		<< "Total cost: " << totalCost << endl ;
	allCosts.push_back(totalCost) ;
	
	/**********************************************************************************************/
	// Traverse greedy path (lowest immediate cost, searching only non-dominated paths)
	cout << "Traversing greedy path...\n" ;
	curLoc = SGPaths[0]->GetVertex() ; // reset path search
	newNodes = SGPaths ; // reset path search
	totalCost = 0 ; // reset path cost
	
	// Assign nodes to first vertex
	curLoc->SetNodes(newNodes) ;
	
	while (curLoc->GetX() != goal->GetX() || curLoc->GetY() != goal->GetY())
	{
		cout << "Current Location: (" << curLoc->GetX() << "," <<  curLoc->GetY() << ")\n" ;
		
		// Extract nodes of current vertex
		newNodes = curLoc->GetNodes() ;
		
		// Identify next vertices
		for (int i = 0; i < newNodes.size(); i++)
		{
			if (!newNodes[i]->GetParent())
				continue ;
			bool newVert = true ;
			for (int j = 0; j < nextVerts.size(); j++)
			{
				
				if ((nextVerts[j]->GetX() == newNodes[i]->GetParent()->GetVertex()->GetX() &&
					nextVerts[j]->GetY() == newNodes[i]->GetParent()->GetVertex()->GetY()) ||
					(nextVerts[j]->GetX() == curLoc->GetX() && nextVerts[j]->GetY() == curLoc->GetY()))
				{
					newVert = false ;
					break ;
				}
			}
			if (newVert)
				nextVerts.push_back(newNodes[i]->GetParent()->GetVertex()) ;
		}
		
		// Identify next vertex path nodes
		for (int i = 0; i < nextVerts.size(); i++)
		{
			tmpNodes.clear() ;
			for (int j = 0; j < newNodes.size(); j++)
			{
				if (!newNodes[j]->GetParent())
					continue ;
				if (nextVerts[i]->GetX() == newNodes[j]->GetParent()->GetVertex()->GetX() &&
					nextVerts[i]->GetY() == newNodes[j]->GetParent()->GetVertex()->GetY())
					tmpNodes.push_back(newNodes[j]->GetParent()) ;
			}
			nextVerts[i]->SetNodes(tmpNodes) ;
		}
		
		// Set cost-to-come for next vertices
		SetTrueEdgeCosts(curLoc, nextVerts, searchGraph) ;
		
		// Rank next vertices according to probability of improvement
		sort(nextVerts.begin(),nextVerts.end(),GreedyComparison) ;
		
		// Move to best vertex and log the cost
		curLoc = nextVerts[0] ;
		totalCost += nextVerts[0]->GetCTC() ;
		
		// Clear vectors for next step
		newNodes.clear() ;
		nextVerts.clear() ;
	}
	
	cout << "Goal vertex (" << curLoc->GetX() << "," <<  curLoc->GetY() << ") reached! "
		<< "Total cost: " << totalCost << endl ;
	allCosts.push_back(totalCost) ;
	
	/**********************************************************************************************/
	// Traverse D* Lite search
	cout << "Traversing dynamic A* path...\n" ;
	totalCost = 0 ;
	totalCost = DynamicAStar(searchGraph, SGPaths[0]->GetVertex(), GSPaths[0]->GetVertex()) ;
	
	allCosts.push_back(totalCost) ;
	
	/**********************************************************************************************/
	// Optimal path cost
	cout << "Traversing optimal path...\n" ;
	totalCost = 0 ;
	totalCost = OptimalAStar(searchGraph, SGPaths[0]->GetVertex(), GSPaths[0]->GetVertex()) ;
	
	allCosts.push_back(totalCost) ;
	
	return allCosts ;
	
}
