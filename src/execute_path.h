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
	
	double c_A0 = A->GetCTC() ; // need to set up mean and var to come
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

double DynamicAStar(Graph * graph, Vertex * source, Vertex * goal) //To avoid looping I need to convert this to get neighbors using nodes or change the vertex get neighbors function
{
	double cost = 0.0 ;
	Vertex * curLoc = source ;
        //make curLoc a node
        
        clock_t t_start = clock() ;
	double t_elapse = 0.0 ;

        //Create search object and perform path search from source to goal
        Search * testSearch = new Search(graph, curLoc, goal) ;
        pathOut pType = BEST ;
        vector<Node *> bestPathsGS = testSearch->PathSearch(pType) ;
        
        // Reverse path
        Node * curNode = bestPathsGS[0]->ReverseList(0) ;
        delete testSearch ;
        testSearch = 0 ;

	
	while ((curLoc->GetX() != goal->GetX() || curLoc->GetY() != goal->GetY()) && t_elapse < 10)
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
		//curLoc = bestPathSG->GetParent()->GetVertex() ;
		curNode = bestPathSG->GetParent() ;
                curLoc = curNode->GetVertex() ;
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
	clock_t t_start = clock() ;
	double t_elapse = 0.0 ;
	Search * testSearch = new Search(graph, source, goal) ;
	pathOut pType = BEST ;
	vector<Node *> bestPathsGS = testSearch->PathSearch(pType) ;
        Node * curNode = bestPathsGS[0]->ReverseList(0) ;
      	Vertex * curLoc = bestPathsGS[0]->GetVertex() ;
	curLoc = curNode->GetVertex() ;
	double cost = 0.0 ;
        stringstream oFileName ;
        oFileName << "../results/optimal" << trialNum << ".txt" ;
        ofstream optimalFile ;
	optimalFile.open(oFileName.str().c_str(), ios::app) ;
	if (bestPathsGS.size() != 0)
	{
		cost = bestPathsGS[0]->GetMeanCost() ;
		cout << "Total cost: " << cost << endl ;
                while (curNode->GetParent())
                {
                        //cout << "Current Location: (" << curLoc->GetX() << "," <<  curLoc->GetY() << ")\n" ;
                        optimalFile << curLoc->GetX() << "," << curLoc->GetY() << "\n";
                        
                        // Move to next vertex and log the cost
                        curNode = curNode->GetParent() ;
                        curLoc = curNode->GetVertex() ;
		        t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
                }
                optimalFile << curLoc->GetX() << "," << curLoc->GetY() << "\n";
	}
	else
	{
		cout << "Failed to reach goal vertex!\n" ;
		cost = DBL_MAX ;
	}
        optimalFile.close() ;
	
	// Reset edge search costs
	ResetEdges(graph) ;
	
	// Delete pointers on the heap
	delete testSearch ;
	testSearch = 0 ;
	
	return cost ;
}

double sampledAStar(Graph * graph, Vertex * source, Vertex * goal, double rags_time){
//      Sampled A* search based on most frequent path chosen
	clock_t t_start = clock() ;
	double t_elapse = 0.0 ;
        int max_search = 1000 ;
        int p_count = 0 ;
        vector< vector<Vertex *> > sampledPaths ;
        stringstream oFileName ;
        oFileName << "../results/sampled" << trialNum << ".txt" ;
        ofstream sampledFile ;
        sampledFile.open(oFileName.str().c_str(), ios::app) ;

        // Sample the cost distributions and run A* repeatedly
        cout << "Searching using Sampled Values... " << endl; 
        while(t_elapse < rags_time && p_count < max_search){

                int seed = rand() % 1000000 ;
	        default_random_engine generator(seed) ;
                // Search using sampled costs of all edges
                ULONG numEdges = graph->GetNumEdges() ;
                for (ULONG i = 0; i < numEdges; i++)
                {
                        graph->GetEdges()[i]->SetSampledCost(generator) ;
                        graph->GetEdges()[i]->SetMeanSearch(graph->GetEdges()[i]->GetSampledCost()) ; //change to the temp values somehow
                        graph->GetEdges()[i]->SetVarSearch(0.0) ;
                }
                
                //Create search object and perform path search from source to goal
                Search * testSearch = new Search(graph, source, goal) ;
                pathOut pType = BEST ;
                vector<Node *> bestPathsGS = testSearch->PathSearch(pType) ;
                Node * curNode = bestPathsGS[0]->ReverseList(0) ;
                Vertex * curLoc = bestPathsGS[0]->GetVertex() ;
                curLoc = curNode->GetVertex() ;
                vector< Vertex * > curPath ;
                if (bestPathsGS.size() != 0)
                {
                        while (curNode->GetParent())
                        {
                                //cout << "Current Location: (" << curLoc->GetX() << "," <<  curLoc->GetY() << ")\n" ;
                                // Move to next vertex and log the cost
                                curNode = curNode->GetParent() ;
                                curLoc = curNode->GetVertex() ;
                                curPath.push_back(curLoc) ;
                        }
                }
                else
                {
                        cout << "Failed to reach goal vertex!\n" ;
                }
                sampledPaths.push_back(curPath) ;
                
                // Reset edge search costs
                ResetEdges(graph) ;

                // Delete pointers on the heap
                delete testSearch ;
                testSearch = 0 ;
                p_count += 1 ;
                t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
        }

        int k = sampledPaths.size();
        int path, max_count = 0;
        for(int i = 0 ; i < k; i++){
                int mycount = std::count (sampledPaths.begin(), sampledPaths.end(), sampledPaths[i]);
                if(mycount > max_count){
                        path = i;
                        max_count = mycount;
                }

        }
        cout << "SAME PATHS  " << max_count << endl;



	double totalCost = 0 ; // reset path cost
// Need to set true costs and actually traverse graph
	cout << "Traversing Sampled A* path...\n" ;
        Vertex * previous_v = source ;//sampledPaths[path][0];
	Edge ** edges = graph->GetEdges() ;
	ULONG numEdges = graph->GetNumEdges() ;

        for(int i = 0; i < sampledPaths[path].size(); i++){
                // find edge
                cout << "Current Location: " << previous_v->GetX() << " " << previous_v->GetY() << endl;
                cout << "Next Location: " << sampledPaths[path][i]->GetX() << " " << sampledPaths[path][i]->GetY() << endl;
                for(int j = 0; j < numEdges; j ++){
                        if(edges[j]->IsEdge(previous_v, sampledPaths[path][i])){
                              previous_v = sampledPaths[path][i];
                              totalCost += edges[j]->GetTrueCost();
                              break;
                        }
                }
        }

        sampledFile <<  p_count << ", " <<  max_count << "\n"; 

        sampledFile.close() ;

        // Reset edge search costs
        ResetEdges(graph) ;

          cout << p_count << " paths sampled, max " << max_count << " same paths, executed cost: " << totalCost << endl;
        return totalCost ;
}


vector<double> executePath(vector< Node*> GSPaths, Graph * searchGraph, double t_nondom)
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
	
        // Initialise timers
        double t_elapse = 0.0 ;
        clock_t t_start = clock() ;

	/**********************************************************************************************/
	// Step through paths
	cout << "Traversing RAGS search paths..." << endl ;
	totalCost = 0 ;
	Vertex * curLoc = SGPaths[0]->GetVertex() ;
	Vertex * goal = GSPaths[0]->GetVertex() ;
	newNodes = SGPaths ;

        // Log RAGS computation time
        t_start = clock() ;


	// Assign nodes to first vertex
	curLoc->SetNodes(newNodes) ;
        stringstream rFileName;
        rFileName << "../results/ragsPath" << trialNum << ".txt";
        ofstream ragsFile;
        ragsFile.open(rFileName.str().c_str(), ios::app);
	
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

                //write loc to file
                ragsFile << curLoc->GetX()<< "," << curLoc->GetY() << "\n";
              
	}
        t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
        double rags_elapse = t_elapse ;

	
	cout << "Goal vertex (" << curLoc->GetX() << "," <<  curLoc->GetY() << ") reached! "
		<< "Total cost: " << totalCost << endl ;
	allCosts.push_back(totalCost) ;
        allCosts.push_back(t_elapse + t_nondom) ;
        cout << "Total RAGS time: " << rags_elapse + t_nondom << endl ;


	
        ragsFile.close() ;
	newNodes.clear() ;
	nextVerts.clear() ;
	


	/**********************************************************************************************/
	// Traverse A* path (lowest mean cost path, no adaptivity)

        stringstream aFileName;
        aFileName << "../results/astarPath" << trialNum << ".txt";
        ofstream astarFile;
        astarFile.open(aFileName.str().c_str(), ios::app);
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

        // Log A* computation time
        t_start = clock() ;

	Node * curNode = SGPaths[lowestCostInd] ;
	curLoc = curNode->GetVertex() ;
        astarFile << curLoc->GetX() << "," << curLoc->GetY() << "\n";
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
                astarFile << curLoc->GetX() << "," << curLoc->GetY() << "\n";
		
		// Clear vectors for next step
		newNodes.clear() ;
		nextVerts.clear() ;
	}

        astarFile.close();
        t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;

	
	cout << "Goal vertex (" << curLoc->GetX() << "," <<  curLoc->GetY() << ") reached! "
		<< "Total cost: " << totalCost << endl ;
	allCosts.push_back(totalCost) ;
        allCosts.push_back(t_elapse) ;

	
	/**********************************************************************************************/
	// Traverse greedy path (lowest immediate cost, searching only non-dominated paths)
	cout << "Traversing greedy path...\n" ;
	curLoc = SGPaths[0]->GetVertex() ; // reset path search
	newNodes = SGPaths ; // reset path search
	totalCost = 0 ; // reset path cost

         // Log greedy computation time
         t_start = clock() ;
     
	
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
	
        t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
	cout << "Goal vertex (" << curLoc->GetX() << "," <<  curLoc->GetY() << ") reached! "
		<< "Total cost: " << totalCost << endl ;
	allCosts.push_back(totalCost) ;
        allCosts.push_back(t_elapse + t_nondom) ;

	
	/**********************************************************************************************/
	// Traverse D* Lite search
	//cout << "Traversing dynamic A* path...\n" ;
	//totalCost = 0 ;
	//totalCost = DynamicAStar(searchGraph, SGPaths[0]->GetVertex(), GSPaths[0]->GetVertex()) ;
	//
	//allCosts.push_back(totalCost) ;


        /**********************************************************************************************/
        //Traverse Sampled A* Search
	cout << "Traversing Sampled A* path...\n" ;
        totalCost = 0 ;
        t_start = clock() ;
        totalCost = sampledAStar(searchGraph, SGPaths[0]->GetVertex(), GSPaths[0]->GetVertex(), (rags_elapse + t_nondom)) ;
        t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;

        allCosts.push_back(totalCost) ;
        allCosts.push_back(t_elapse) ;


	/**********************************************************************************************/
	// Optimal path cost
	cout << "Traversing optimal path...\n" ;
	totalCost = 0 ;
	totalCost = OptimalAStar(searchGraph, SGPaths[0]->GetVertex(), GSPaths[0]->GetVertex()) ;
	
	allCosts.push_back(totalCost) ;
	
	return allCosts ;
	
}

bool ComputeImprovementProbabilityNonDynamic(Vertex * A, Vertex * B)
{
	vector<Node *> ANodes = A->GetNodes();
	vector<Node *> BNodes = B->GetNodes();
	double muA = A->GetCV0Mean() ;
	double varA = A->GetCV0Var() ;
	double muB = B->GetCV0Mean() ;
	double varB = B->GetCV0Var() ;
	
	double max_3sig = ANodes[0]->GetMeanCTG() + muA + 3*(ANodes[0]->GetVarCTG() + varA) ;
	double min_3sig = ANodes[0]->GetMeanCTG() + muA - 3*(ANodes[0]->GetVarCTG() + varA) ;
	for (int i = 0; i < ANodes.size(); i++)
	{
		if (max_3sig < ANodes[i]->GetMeanCTG() + muA + 3*(ANodes[i]->GetVarCTG() + varA))
			max_3sig = ANodes[i]->GetMeanCTG() + muA + 3*(ANodes[i]->GetVarCTG() + varA) ;
		if (min_3sig > ANodes[i]->GetMeanCTG() + muA - 3*(ANodes[i]->GetVarCTG() + varA))
			min_3sig = ANodes[i]->GetMeanCTG() + muA - 3*(ANodes[i]->GetVarCTG() + varA) ;
	}
	for (int i = 0; i < BNodes.size(); i++)
	{
		if (max_3sig < BNodes[i]->GetMeanCTG() + muB + 3*(BNodes[i]->GetVarCTG() + varB))
			max_3sig = BNodes[i]->GetMeanCTG() + muB + 3*(BNodes[i]->GetVarCTG() + varB) ;
		if (min_3sig > BNodes[i]->GetMeanCTG() + muB - 3*(BNodes[i]->GetVarCTG() + varB))
			min_3sig = BNodes[i]->GetMeanCTG() + muB - 3*(BNodes[i]->GetVarCTG() + varB) ;
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
			double mu_Ai = ANodes[i]->GetMeanCTG() + muA ;
			double sig_Ai = ANodes[i]->GetVarCTG() + varA ;
			double p_cA1 = (1/(sig_Ai*sqrt(2*pi)))*exp(-(pow(x[k]-mu_Ai,2))/(2*pow(sig_Ai,2))) ;
			double p_cA2 = 1.0 ;
			for (int j = 0; j < ANodes.size(); j++)
			{
				double mu_Aj = ANodes[j]->GetMeanCTG() + muA ;
				double sig_Aj = ANodes[j]->GetVarCTG() + varA ;
				if (j != i)
					p_cA2 *= 0.5*erfc((x[k]-mu_Aj)/(sig_Aj*sqrt(2))) ;
			}
			p_cAi += p_cA1*p_cA2 ;
		}
		double p_cBi = 1.0 ;
		for (int i = 0; i < BNodes.size(); i++)
		{
			double mu_Bi = BNodes[i]->GetMeanCTG() + muB ;
			double sig_Bi = BNodes[i]->GetVarCTG() + varB ;
			p_cBi *= 0.5*erfc((x[k]-mu_Bi)/(sig_Bi*sqrt(2))) ;
		}
		pImprove += (p_cAi)*(1-p_cBi)*dx ;
	}
	
	return (pImprove<=0.5);
}

void executeNonDynamicRAGS(vector< Node*> GSPaths, Graph * searchGraph){
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
	// Step through RAGS non-dynamic path
	cout << "Traversing non-dynamic RAGS path..." << endl ;
	Vertex * curLoc = SGPaths[0]->GetVertex() ;
	Vertex * goal = GSPaths[0]->GetVertex() ;
	newNodes = SGPaths ;

	// Assign nodes to first vertex
	curLoc->SetNodes(newNodes) ;
  stringstream rFileName;
  rFileName << "../results/ragsPathNonDynamic" << trialNum << ".txt";
  ofstream ragsFileNonDyanmic;
  ragsFileNonDyanmic.open(rFileName.str().c_str(), ios::app);
	
	while (curLoc->GetX() != goal->GetX() || curLoc->GetY() != goal->GetY())
	{
		cout << "Current Location: (" << curLoc->GetX() << "," <<  curLoc->GetY() << ")\n" ;
		
		// Extract nodes of current vertex
		newNodes = curLoc->GetNodes() ;
		
		// Identify next vertices
		for (int i = 0; i < newNodes.size(); i++)
		{
		  double CTGMu = newNodes[i]->GetMeanCTG() ;
		  double CTGVar = newNodes[i]->GetVarCTG() ;
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
			if (newVert){
				double diffMu = CTGMu - newNodes[i]->GetParent()->GetMeanCTG() ;
				double diffVar = CTGVar - newNodes[i]->GetParent()->GetVarCTG() ;
				newNodes[i]->GetParent()->GetVertex()->SetCV0Mean(diffMu) ;
				newNodes[i]->GetParent()->GetVertex()->SetCV0Var(diffVar) ;
				nextVerts.push_back(newNodes[i]->GetParent()->GetVertex()) ;
			}
		}
		
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
		
		// Rank next vertices according to probability of improvement
		sort(nextVerts.begin(),nextVerts.end(),ComputeImprovementProbabilityNonDynamic) ;
		
		// Move to best vertex and log the cost
		curLoc = nextVerts[0] ;
		
		// Clear vectors for next step
		newNodes.clear() ;
		nextVerts.clear() ;

    //write loc to file
    ragsFileNonDyanmic << curLoc->GetX()<< "," << curLoc->GetY() << "\n";
              
	}

  ragsFileNonDyanmic.close() ;
	newNodes.clear() ;
	nextVerts.clear() ;
}
