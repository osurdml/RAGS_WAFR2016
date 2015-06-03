#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <queue>
#include <vector>
#include <functional>
#include <cmath>
#include <math.h>
#include <float.h>
#include <limits.h>
using namespace std ;

typedef unsigned long int ULONG ;
const double pi = 3.14159265358979323846264338328 ;

#include "config.h"
#include "vertex.h"
#include "edge.h"
#include "node.h"
#include "graph.h"
#include "queue.h"
#include "search.h"

void DefineGraph(vector< vector<double> > & vertVec, vector< vector<double> > & edgeVec) 
{
	ifstream verticesFile("sector_vertices1.txt") ;

	cout << "Reading vertices from file..." ;
	vector<double> v(2) ;
	string line ;
	while (getline(verticesFile,line))
	{
		stringstream lineStream(line) ;
		string cell ;
		int i = 0 ;
		while (getline(lineStream,cell,','))
		{
			v[i++] = atof(cell.c_str()) ;
		}
		vertVec.push_back(v) ;
	}
	cout << "complete.\n" ;
	
	ifstream edgesFile("sector_edges1.txt") ;
	
	cout << "Reading edges from file..." ;
	vector<double> e(4) ;
	while (getline(edgesFile,line))
	{
		stringstream lineStream(line) ;
		string cell ;
		int i = 0 ;
		while (getline(lineStream,cell,','))
		{
			e[i++] = atof(cell.c_str()) ;
		}
		edgeVec.push_back(e) ;
	}
	cout << "...complete.\n" ;
}

void DefineWorld(vector< vector<bool> > & obstacles, vector< vector<int> > & membership)
{
	ifstream obstaclesFile("obstacles.txt") ;
	
	cout << "Reading obstacle map from file..." ;
	vector<bool> obs(25) ;
	string line ;
	int lineCount = 0 ;
	while (getline(obstaclesFile,line))
	{
		stringstream lineStream(line) ;
		string cell ;
		int i = 0 ;
		while (getline(lineStream,cell,','))
		{
			if (atoi(cell.c_str()) == 1)
				obs[i++] = true ;
			else
				obs[i++] = false ;
		}
		obstacles.push_back(obs) ;
	}
	cout << "complete.\n" ;
	
	ifstream sectorsFile("membership.txt") ;
	
	cout << "Reading sector map from file..." ;
	while (getline(sectorsFile,line))
	{
		stringstream lineStream(line) ;
		string cell ;
		vector<int> mem ;
		while (getline(lineStream,cell,','))
		{
			mem.push_back(atoi(cell.c_str())) ;
		}
		membership.push_back(mem) ;
	}
	cout << "complete.\n" ;
}

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

double ComputeImprovementProbability(double c_A0, double c_B0, vector<double> mu_A, vector<double> sig_A, vector<double> mu_B, vector<double> sig_B)
{ // The longer vectors should be set to B
	double max_3sig = mu_A[0] + 3*sig_A[0] ;
	double min_3sig = mu_A[0] - 3*sig_A[0] ;
	for (int i = 0; i < mu_A.size(); i++)
	{
		if (max_3sig < mu_A[i]+3*sig_A[i])
			max_3sig = mu_A[i]+3*sig_A[i] ;
		if (min_3sig > mu_A[i]-3*sig_A[i])
			min_3sig = mu_A[i]-3*sig_A[i] ;
	}
	for (int i = 0; i < mu_B.size(); i++)
	{
		if (max_3sig < mu_B[i]+3*sig_B[i])
			max_3sig = mu_B[i]+3*sig_B[i] ;
		if (min_3sig > mu_B[i]-3*sig_B[i])
			min_3sig = mu_B[i]-3*sig_B[i] ;
	}
	
	int n = 10000 ;
	vector<double> x = linspace(min_3sig,max_3sig,n) ;
	double dx = x[1]-x[0] ;
	double pImprove = 0.0 ;
	for (int k = 0; k < x.size(); k++)
	{
		double p_cAi = 0.0 ;
		for (int i = 0; i < mu_A.size(); i++)
		{
			double p_cA1 = (1/(sig_A[i]*sqrt(2*pi)))*exp(-(pow(x[k]-mu_A[i],2))/(2*pow(sig_A[i],2))) ;
			double p_cA2 = 1.0 ;
			for (int j = 0; j < mu_A.size(); j++)
			{
				if (j != i)
					p_cA2 *= 0.5*erfc((x[k]-mu_A[j])/(sig_A[j]*sqrt(2))) ;
			}
			p_cAi += p_cA1*p_cA2 ;
		}
		double p_cBi = 1.0 ;
		for (int i = 0; i < mu_B.size(); i++)
			p_cBi *= 0.5*erfc((x[k]-(c_B0-c_A0)-mu_B[i])/(sig_B[i]*sqrt(2))) ;
		pImprove += (p_cAi)*(1-p_cBi)*dx ;
	}
	return pImprove;
}

int main()
{
	cout << "Test program...\n" ;
	
	// Create vectors and edges in graph from text files
	vector< vector<double> > vertVec ;
	vector< vector<double> > edgeVec ;
	DefineGraph(vertVec, edgeVec) ;
	
	// Create graph
	Graph * testGraph = new Graph(vertVec, edgeVec) ;
	Vertex * sourceSec = testGraph->GetVertices()[0] ; // top-left sector
	Vertex * goalSec = testGraph->GetVertices()[testGraph->GetNumVertices()-1] ; // bottom-right sector
	
	//Create search object and perform path search from source to goal
	cout << "Creating search object..." ;
	Search * testSearch = new Search(testGraph, sourceSec, goalSec) ;
	cout << "complete.\n" ;
	
	cout << "Performing path search from (" <<  sourceSec->GetX() << "," << sourceSec->GetY() << ") to (" ;
	cout << goalSec->GetX() << "," << goalSec->GetY() << ")...\n" ;
	pathOut pType = ALL ;
	vector<Node *> bestPaths = testSearch->PathSearch(pType) ;
	cout << "Path search complete.\n" ;
	
	vector<Node *> bestPathsReverse(bestPaths.size()) ;
	if (bestPaths.size() != 0)
	{
		for (ULONG i = 0; i < (ULONG)bestPaths.size(); i++)
		{
			bestPathsReverse[i] = bestPaths[i]->ReverseList(0) ;
			cout << "Path " << i << endl ;
			bestPathsReverse[i]->DisplayPath() ;
		}
	}
	
	delete testGraph ;
	testGraph = 0 ;
	delete testSearch ;
	testSearch = 0 ;
	
	return 0 ;
}
