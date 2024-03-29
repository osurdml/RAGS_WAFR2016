// Edge class to contain mean and variance of cost along an edge
#include <random>
using namespace std;
class Edge
{
	public:
		Edge(Vertex * v1, Vertex * v2, double cost, double var) ;
		~Edge() ;
		
		Vertex * GetVertex1() const {return itsVertex1 ;}
		void SetVertex1(Vertex * vertex1) {itsVertex1 = vertex1 ;}
		Vertex * GetVertex2() const {return itsVertex2 ;}
		void SetVertex2(Vertex * vertex2) {itsVertex2 = vertex2 ;}
		double GetMeanCost() const {return itsMeanCost ;}
		void SetMeanCost(double cost) {itsMeanCost = cost ;}
		double GetVarCost() const {return itsVarCost ;}
		void SetVarCost(double var) {itsVarCost = var ;}
		double GetTrueCost() {return itsTrueCost ;}
		void SetTrueCost(default_random_engine generator) ;
		double GetSampledCost() {return itsSampledCost ;}
		void SetSampledCost(default_random_engine generator) ;
		double GetMeanSearch() const {return itsMeanSearch ;}
		void SetMeanSearch(double cost) {itsMeanSearch = cost ;}
		double GetVarSearch() const {return itsVarSearch ;}
		void SetVarSearch(double var) {itsVarSearch = var ;}
		bool IsEdge(Vertex * v1, Vertex * v2);
	private:
		Vertex * itsVertex1 ;
		Vertex * itsVertex2 ;
		double itsMeanCost ;
		double itsVarCost ;
		double itsTrueCost ;
		double itsSampledCost ;
		double itsMeanSearch ; // Actual value used in search
		double itsVarSearch ; // Actual value used in search
} ;

Edge::Edge(Vertex * v1, Vertex * v2, double cost, double var)
{
	itsVertex1 = v1 ;
	itsVertex2 = v2 ;
	itsMeanCost = cost ;
	itsVarCost = var ;
	itsMeanSearch = cost ;
	itsVarSearch = var ;
}

Edge::~Edge()
{
	delete itsVertex1 ;
	itsVertex1 = 0 ;
	delete itsVertex2 ;
	itsVertex2 = 0 ;
}

void Edge::SetTrueCost(default_random_engine generator)
{
	double diff_x = itsVertex1->GetX() - itsVertex2->GetX() ;
	double diff_y = itsVertex1->GetY() - itsVertex2->GetY() ;
	double diff = sqrt(pow(diff_x,2) + pow(diff_y,2)) ;
	normal_distribution<double> distribution(itsMeanCost,itsVarCost) ;
	itsTrueCost = distribution(generator) ;
	if (itsTrueCost < diff)
		itsTrueCost = diff ;
}
void Edge::SetSampledCost(default_random_engine generator)
{
	double diff_x = itsVertex1->GetX() - itsVertex2->GetX() ;
	double diff_y = itsVertex1->GetY() - itsVertex2->GetY() ;
	double diff = sqrt(pow(diff_x,2) + pow(diff_y,2)) ;
	normal_distribution<double> distribution(itsMeanCost,itsVarCost) ;
	itsSampledCost = distribution(generator) ;
	if (itsSampledCost < diff)
		itsSampledCost = diff ;
}

bool Edge::IsEdge(Vertex * vertex1, Vertex * vertex2){
        if(vertex1->GetX() == itsVertex1->GetX() &&  vertex1->GetY() == itsVertex1->GetY() && vertex2->GetX() == itsVertex2->GetX() &&  vertex2->GetY() == itsVertex2->GetY()){
          return true;
        }
        else{
          return false;
        }
}
