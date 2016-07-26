#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <vector>
#include <queue>
#include <vector>
#include <functional>
#include <cmath>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <stdlib.h>
#include <typeinfo>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std ;

typedef unsigned long int ULONG ;
int trialNum ;

#include "config.h"
#include "vertex.h"
#include "edge.h"
#include "node.h"
#include "graph.h"
#include "queue.h"
#include "search.h"
#include "path.h"
#include "execute_path.h"

cv::Mat ImportImage(int imageNum){
        std::ostringstream imageName;
        imageName << "../config/images/image" << imageNum << ".png";
        cout << imageName.str().c_str() << endl;
        cv::Mat img = cv::imread(imageName.str().c_str(), 0);
        // imshow( "Display window", img );                   // Show our image inside it.
        // cv::waitKey(0);                                          // Wait for a keystroke in the window
        return img;
}


void DefineGraph(vector< vector<double> > & vertVec, vector< vector<double> > & edgeVec) 
{
	ifstream verticesFile("../config/vertices_dom_test.txt") ;

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

	ifstream edgesFile("../config/edges_dom_test.txt") ;

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
	ifstream obstaclesFile("../config/obstacles.txt") ;

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

	ifstream sectorsFile("../config/membership.txt") ;

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

vector< vector< double > > makeVertices(double x, double y, int numVerts){
	vector< vector< double > > vertices(numVerts, vector<double>(2));

	srand (time(NULL));
	double vertx, verty;
	int xx = x;
	int yy = y;
	double testx, testy;

	for(int i = 0; i < numVerts; i++){
		if (i == 0)
		{
			vertices[i][0] = 0 ;
			vertices[i][1] = 0 ;
		}
		else if (i == numVerts-1)
		{
			vertices[i][0] = x ;
			vertices[i][1] = y ;
		}
		else
		{
			vertx = rand() % xx;
			verty = rand() % yy;
			vertices[i][0] = vertx ;
			vertices[i][1] = verty ;
		}
	}

	// Write vertices to txt file
	stringstream vFileName ;
	vFileName << "../results/vertices" << trialNum << ".txt" ;

	ofstream vertsFile ;
	vertsFile.open(vFileName.str().c_str(), ios::app) ;

	for (ULONG i = 0; i < vertices.size(); i++)
	{
		vertsFile << vertices[i][0] << "," << vertices[i][1] << "\n" ;
	}
        vertsFile << "\n";
	vertsFile.close() ;

	return vertices;
}

int main()
{
	cout << "Trial number: " ;
	cin >> trialNum ;
	
	cout << "Test program " << trialNum << "...\n" ;
	
        int iter = 1;
	int numVerts = 150 ;

        // Write Configuration
        stringstream vFileName ;
        vFileName << "../results/config" << trialNum << ".txt" ;
        ofstream confFile ;
        confFile.open(vFileName.str().c_str()) ;

        confFile << "Satellite Data " << "\n" << "numVerts: " << numVerts << "\n";
        confFile.close();
        while (iter <= 100){
            bool loop = true ;
            srand(time(NULL));
            while (loop)
            {
                    cv::Mat img = ImportImage(iter);
                    vector< double > size;
                    size.push_back(img.size().width-1);
                    size.push_back(img.size().height-1);
                    vector< vector< double > > vertVec2;
                    double x, y, radius;
                    x = size[0];
                    y = size[1];
                    cout << "Generating Random Vertices in " << x << " by " << y << endl;
                    vertVec2 = makeVertices(x,y,numVerts);
                    radius = sqrt((6.0/pi)*x*y*(log((double)numVerts)/(double)numVerts)) ;
                    cout << "Connecting with radius " << radius << endl;
                    //Graph * testGraph = new Graph(vertVec, edgeVec) ;
                    Graph * testGraph = new Graph(vertVec2, radius, img);
                    //Graph * testGraph = DefineGraph(vertVec2, edgeVec);
                    Vertex * sourceSec = testGraph->GetVertices()[0] ; // top-left sector
                    Vertex * goalSec = testGraph->GetVertices()[testGraph->GetNumVertices()-1] ; // bottom-right sector

                    //Create search object and perform path search from source to goal
                    cout << "Creating search object..." ;
                    Search * testSearch = new Search(testGraph, sourceSec, goalSec) ;
                    cout << "complete.\n" ;

                    cout << "Performing path search from (" <<  sourceSec->GetX() << "," << sourceSec->GetY() << ") to (" ;
                    cout << goalSec->GetX() << "," << goalSec->GetY() << ")...\n" ;
                    pathOut pTypeCheck = BEST ;

                    // Log initial search time
                    double t_elapse = 0.0 ;
                    clock_t t_start = clock() ;
                    vector<Node *> bestPathSingle = testSearch->PathSearch(pTypeCheck) ;
                    t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
                    
                    // Write to txt file
                    stringstream sFileName ;
                    sFileName << "../results/searchTime" << trialNum << ".txt" ;
                    ofstream searchFile ;
                    searchFile.open(sFileName.str().c_str()) ;          
                    
                    searchFile << "Best path search time: " << t_elapse << "\n" ;


                    pathOut pType = ALL ;
                    // Log non-dominated path set search time
                    t_start = clock() ;
                    vector<Node *> bestPaths = testSearch->PathSearch(pType) ;
                    t_elapse = (float)(clock() - t_start)/CLOCKS_PER_SEC ;
                    
                    searchFile << "Non-dominated path set search time: " << t_elapse ;
                    searchFile.close() ;
                    cout << "Path search complete, " << bestPaths.size() << " paths found.\n" ;

                    // Write paths to file
                    if (bestPaths.size() != 0)
                    {
                            stringstream pFileName ;
                            pFileName << "../results/bestPaths" << trialNum << ".txt" ;
                            
                            ofstream pathsFile ;
                            pathsFile.open(pFileName.str().c_str()) ;
                            
                            for (ULONG i = 0; i < (ULONG)bestPaths.size(); i++)
                            {
                                    pathsFile << "Path " << i << endl ;
                                    Node * curNode = bestPaths[i] ;
                                    while (curNode->GetParent())
                                    {
                                            pathsFile << "(" << curNode->GetVertex()->GetX() << ","
                                                    << curNode->GetVertex()->GetY() << ")\n" ;
                                            curNode = curNode->GetParent() ;
                                    }
                                    pathsFile << "(" << curNode->GetVertex()->GetX() << ","
                                            << curNode->GetVertex()->GetY() << ")\n\n" ;				
                            }
                            pathsFile.close() ;
                    }

                    if (bestPaths.size() < 30 || bestPaths.size() > 200)
                    {
                            delete testGraph ;
                            testGraph = 0 ;
                            delete testSearch ;
                            testSearch = 0 ;
                            continue ;
                    }
                    else
                            loop = false ;
                    
                    // Execute path
                    vector< double > costs ;
                    int totalStatRuns = 1 ;
                    vector< vector< double > > allCosts(totalStatRuns, vector<double>(5)) ;

                    for(int numStatRuns = 0; numStatRuns < totalStatRuns; numStatRuns++)
                    {
                            // Randomly select seed
                            //int seed = rand() % 1000000 ;
                            
                            // Assign true edge costs
                            AssignTrueEdgeCosts(testGraph, numStatRuns+1) ;
                            //AssignTrueEdgeCosts(testGraph, seed+1) ;
                            
                            costs = executePath(bestPaths, testGraph, t_elapse);
                            allCosts[numStatRuns] = costs ;
                    }

                    // Write vertices to txt file
                    stringstream cFileName ;
                    stringstream tFileName ;
                    cFileName << "../results/pathCosts" << trialNum << ".txt" ;
                    tFileName << "../results/compTime" << trialNum << ".txt" ;
                    
                    ofstream costsFile ;
                    ofstream timeFile ;
                    costsFile.open(cFileName.str().c_str(), ios::app) ;
                    timeFile.open(tFileName.str().c_str()) ;
                    
                    for (ULONG i = 0; i < allCosts.size(); i++)
                    {
                            for (int j = 0; j < allCosts[i].size(); j++)
                            {
                                    if ( j%2 == 0) 
                                            costsFile << allCosts[i][j] << "," ;
                                    else
                                            timeFile << allCosts[i][j] << "," ;
                            }
                            costsFile << "\n" ;
                            timeFile << "\n" ;
                    }
                    costsFile.close() ;
                    timeFile.close() ;

                    /*//Read in membership and obstacle text files
                    vector< vector<bool> > obstacles ;
                    vector< vector<int> > membership ;
                    DefineWorld(obstacles, membership) ;

                    //Create path object to plan low level path
                    Vertex * sourcePath = new Vertex(2.0,3.0) ;
                    Vertex * goalPath = new Vertex(48.0,17.0) ;
                    cout << "Performing low level path search from (" << sourcePath->GetX() << "," ;
                    cout << sourcePath->GetY() << ") to (" << goalPath->GetX() << "," ;
                    cout << goalPath->GetY() << ")...\n" ;
                    Path * testPath = new Path(obstacles, membership, bestPaths[0], sourcePath, goalPath) ;
                    Node * lowLevelPath = testPath->ComputePath(8) ;
                    cout << "Low level path search complete.\n" ;*/

                    delete testGraph ;
                    testGraph = 0 ;
                    delete testSearch ;
                    testSearch = 0 ;
                    //delete sourcePath ;
                    //sourcePath = 0 ;
                    //delete goalPath ;
                    //goalPath = 0 ;
                    cout << "FINISHED LOOP " << iter << endl;
            }
        iter += 1;
        }
        cout << "Trial Number: " << trialNum << " Completed" << endl;
	return 0 ;
}
