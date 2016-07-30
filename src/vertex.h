// Vertex class to contain location of vertices
class Node;
class Vertex
{
	public:
		Vertex(double x, double y) ;
		~Vertex() {}
		
		double GetX() const {return itsX ;}
		void SetX(double x) {itsX = x ;}
		double GetY() const {return itsY ;}
		void SetY(double y) {itsY = y ;}
		double GetCTC()const{return itsCTC ;}
		void SetCTC(double cost_to_come) {itsCTC = cost_to_come ;}
		void SetNodes(vector<Node *> NewNodes) {itsNodes = NewNodes ;}
		vector<Node *> GetNodes() {return itsNodes ;}
		void SetActualCost(double ac) {itsActualCost = ac;}
		double GetActualCost() const {return itsActualCost ;}
		void SetCV0Mean(double mu){cV0[0] = mu ;}
		double GetCV0Mean(){return cV0[0] ;}
		void SetCV0Var(double var){cV0[1] = var ;}
		double GetCV0Var(){return cV0[1] ;}


	private:
		double itsActualCost ;
		double itsX ;
		double itsY ;
		double itsCTC ;
		vector<double> cV0 ;
		vector<Node *> itsNodes ;
};

Vertex::Vertex(double x, double y)
{
	itsX = x ;
	itsY = y ;
	cV0.push_back(0.0) ;
	cV0.push_back(0.0) ;
}
