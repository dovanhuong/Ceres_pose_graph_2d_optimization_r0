//*************************************************************************//
// ------>      Huong Do Van - email: huong.dovan@kist.re.kr       <------ //
// ------>   Ceres Solver Optimization - Lecturer: Dr.Hwasup Lim   <------ //
// ------>       Korea Institute of Science and Technology         <------ //
//*************************************************************************//

#include <iostream>
#include <string>
#include <fstream>
#include <cstdlib>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

#define ODOMETRY_EDGE 0
#define CLOSURE_EDGE 1
#define BOGUS_EDGE 2
#define drand48() ((double)rand()/RAND_MAX)

// Class to represent Nodes
class Node
{
    public:
    Node(int index, double x, double y, double theta)
    {
        this->index = index;
        p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = theta;
    }
    int index;
    double *p;
};

// Class to represent Edges
class Edge
{
public:
    // Type:
    // 0 : Odometry edge
    // 1 : Loop CLosure Edge
    // 2 : Bogus Edge
    Edge(const Node* a, const Node* b, int edge_type )
    {
        this->a = a;
        this->b = b;
        this->edge_type = edge_type;
    }

    void setEdgePose( double x, double y, double theta )
    {
        this->x = x;
        this->y = y;
        this->theta = theta;
    }

    void setInformationMatrix( double I11, double  I12, double  I13, double I22, double I23, double I33 )
    {
        this->I11 = I11;
        this->I12 = I12;
        this->I13 = I13;
        this->I22 = I22;
        this->I23 = I23;
        this->I33 = I33;
    }

    const Node *a, *b;
    double x, y, theta;
    double I11, I12, I13, I22, I23, I33;
    int edge_type;
};

class ReadG2O
{
public:
    ReadG2O(const string& fName)
    {
        //Read the file in g2o format
        fstream fp;
        fp.open(fName.c_str(), ios::in);

        string line;
        int v = 0;
        int e = 0;
        while( std::getline(fp, line) )
        {
            vector<string> words;
            boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
            if( words[0].compare( "VERTEX_SE2") == 0 )
            {
                v++;
                int node_index = boost::lexical_cast<int>( words[1] );
                double x = boost::lexical_cast<double>( words[2] );
                double y = boost::lexical_cast<double>( words[3] );
                double theta = boost::lexical_cast<double>( words[4] );

                Node * node = new Node(node_index, x, y, theta);
                nNodes.push_back( node );
            }

            if( words[0].compare( "EDGE_SE2") == 0 )
            {
                //cout << e << words[0] << endl;
                int a_indx = boost::lexical_cast<int>( words[1] );
                int b_indx = boost::lexical_cast<int>( words[2] );

                double dx = boost::lexical_cast<double>( words[3] );
                double dy = boost::lexical_cast<double>( words[4] );
                double dtheta = boost::lexical_cast<double>( words[5] );

                double I11, I12, I13, I22, I23, I33;
                I11 = boost::lexical_cast<double>( words[6] );
                I12 = boost::lexical_cast<double>( words[7] );
                I13 = boost::lexical_cast<double>( words[8] );
                I22 = boost::lexical_cast<double>( words[9] );
                I23 = boost::lexical_cast<double>( words[10] );
                I33 = boost::lexical_cast<double>( words[11] );

                if( abs(a_indx - b_indx) < 5 )
                {
                  Edge * edge = new Edge( nNodes[a_indx], nNodes[b_indx], ODOMETRY_EDGE );
                  edge->setEdgePose(dx, dy, dtheta);
                  edge->setInformationMatrix(I11, I12, I13, I22, I23, I33);
                  nEdgesOdometry.push_back(edge);
                }
                else
                {
                  Edge * edge = new Edge( nNodes[a_indx], nNodes[b_indx], CLOSURE_EDGE );
                  edge->setEdgePose(dx, dy, dtheta);
                  edge->setInformationMatrix(I11, I12, I13, I22, I23, I33);
                  nEdgesClosure.push_back(edge);
                }
                e++;
            }
        }
    }

    // Write nodes to file to be visualized with python script
    void writePoseGraph_nodes( const string& fname )
    {
      cout << "Write_Pose_Graph Nodes at location: " << fname << endl;
      fstream fp;
      fp.open( fname.c_str(), ios::out );
      for( int i=0 ; i<this->nNodes.size() ; i++ )
      {
        fp << nNodes[i]->index << " " << nNodes[i]->p[0] << " " << nNodes[i]->p[1] << " " << nNodes[i]->p[2]  << endl;
      }
    }

    void writePoseGraph_edges( const string& fname )
    {
      cout << "Write_Pose_Graph Edges at location : "<< fname << endl;
      fstream fp;
      fp.open( fname.c_str(), ios::out );
      write_edges( fp, this->nEdgesOdometry );
      write_edges( fp, this->nEdgesClosure );
      write_edges( fp, this->nEdgesBogus );
    }

    // Adding Bogus edges as described in Vertigo paper
    void add_random_C(int count )
    {
        int MIN = 0 ;
        int MAX = nNodes.size();

        for( int i = 0 ; i<count ; i++ )
        {
            int a = rand() % MAX;
            int b = rand() % MAX;
            cout << a << "<--->" << b << endl;
            Edge * edge = new Edge( nNodes[a], nNodes[b], BOGUS_EDGE );
            edge->setEdgePose( rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX );
            nEdgesBogus.push_back( edge );
        }
    }

//private:
    vector<Node*> nNodes;         //storage for node
    vector<Edge*> nEdgesOdometry; //storage for edges - odometry
    vector<Edge*> nEdgesClosure;  //storage for edges - odometry
    vector<Edge*> nEdgesBogus;    //storage for edges - odometry

    void write_edges( fstream& fp, vector<Edge*>& vec )
    {
      for( int i=0 ; i<vec.size() ; i++ )
      {
        fp << vec[i]->a->index << " " << vec[i]->b->index << " " << vec[i]->edge_type << endl;
      }
    }
};
// Odometry Residue
struct OdometryResidue
{
    // Observation for the edge
    OdometryResidue(double dx, double dy, double dtheta)
    {
        this->dx = dx;
        this->dy = dy;
        this->dtheta = dtheta;
        // make a_Tcap_b
        {
          double cos_t = cos( this->dtheta );
          double sin_t = sin( this->dtheta );
          a_Tcap_b(0,0) = cos_t;
          a_Tcap_b(0,1) = -sin_t;
          a_Tcap_b(1,0) = sin_t;
          a_Tcap_b(1,1) = cos_t;
          a_Tcap_b(0,2) = this->dx;
          a_Tcap_b(1,2) = this->dy;

          a_Tcap_b(2,0) = 0.0;
          a_Tcap_b(2,1) = 0.0;
          a_Tcap_b(2,2) = 1.0;
      }

    }
    // Define the residue for each edge. P1 and P2 are 3-vectors representing state of the node ie. x,y,theta
    template <typename T>
    bool operator()(const T* const P1, const T* const P2, T* e) const
    {
        // Convert P1 to T1 ^w_T_a
        Eigen::Matrix<T,3,3> w_T_a;
        {
          T cos_t = T(cos( P1[2] ));
          T sin_t = T(sin( P1[2] ));
          w_T_a(0,0) = cos_t;
          w_T_a(0,1) = -sin_t;
          w_T_a(1,0) = sin_t;
          w_T_a(1,1) = cos_t;
          w_T_a(0,2) = P1[0];
          w_T_a(1,2) = P1[1];

          w_T_a(2,0) = T(0.0);
          w_T_a(2,1) = T(0.0);
          w_T_a(2,2) = T(1.0);
      }
        // Convert P2 to T2 ^w_T_a
        Eigen::Matrix<T,3,3> w_T_b;
        {
          T cos_t = cos( P2[2] );
          T sin_t = sin( P2[2] );
          w_T_b(0,0) = cos_t;
          w_T_b(0,1) = -sin_t;
          w_T_b(1,0) = sin_t;
          w_T_b(1,1) = cos_t;
          w_T_b(0,2) = P2[0];
          w_T_b(1,2) = P2[1];

          w_T_b(2,0) = T(0.0);
          w_T_b(2,1) = T(0.0);
          w_T_b(2,2) = T(1.0);
      }

        // cast from double to T
        Eigen::Matrix<T, 3, 3> T_a_Tcap_b;
        T_a_Tcap_b <<   T(a_Tcap_b(0,0)), T(a_Tcap_b(0,1)),T(a_Tcap_b(0,2)),
                        T(a_Tcap_b(1,0)), T(a_Tcap_b(1,1)),T(a_Tcap_b(1,2)),
                        T(a_Tcap_b(2,0)), T(a_Tcap_b(2,1)),T(a_Tcap_b(2,2));

        // now we have :: w_T_a, w_T_b and a_Tcap_b
        // compute pose difference
        Eigen::Matrix<T,3,3> diff = T_a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);

        e[0] = diff(0,2);
        e[1] = diff(1,2);
        e[2] = asin( diff(1,0) );

        return true;
    }

    double dx;
    double dy;
    double dtheta;
    Eigen::Matrix<double,3,3> a_Tcap_b;

    static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta){
        return (new ceres::AutoDiffCostFunction<OdometryResidue, 3, 3, 3>(
            new OdometryResidue(dx, dy, dtheta)));
    };
};

// Switchable Loop Closure Residue
struct SwitchableClosureResidue
{
    // Observation for the edge
    SwitchableClosureResidue(double dx, double dy, double dtheta, double s_prior )
    {
        this->dx = dx;
        this->dy = dy;
        this->dtheta = dtheta;
        this->s_prior = s_prior;

        // make a_Tcap_b
        {
          double cos_t = cos( this->dtheta );
          double sin_t = sin( this->dtheta );
          a_Tcap_b(0,0) = cos_t;
          a_Tcap_b(0,1) = -sin_t;
          a_Tcap_b(1,0) = sin_t;
          a_Tcap_b(1,1) = cos_t;
          a_Tcap_b(0,2) = this->dx;
          a_Tcap_b(1,2) = this->dy;

          a_Tcap_b(2,0) = 0.0;
          a_Tcap_b(2,1) = 0.0;
          a_Tcap_b(2,2) = 1.0;
      }
    }

    // Define the residue for each edge. P1 and P2 are 3-vectors representing state of the node ie. x,y,theta
    template <typename T>
    bool operator()(const T* const P1, const T* const P2, const T* const s, T* e) const
    {
        // Convert P1 to T1 ^w_T_a
        Eigen::Matrix<T,3,3> w_T_a;
        {
          T cos_t = T(cos( P1[2] ));
          T sin_t = T(sin( P1[2] ));
          w_T_a(0,0) = cos_t;
          w_T_a(0,1) = -sin_t;
          w_T_a(1,0) = sin_t;
          w_T_a(1,1) = cos_t;
          w_T_a(0,2) = P1[0];
          w_T_a(1,2) = P1[1];

          w_T_a(2,0) = T(0.0);
          w_T_a(2,1) = T(0.0);
          w_T_a(2,2) = T(1.0);
        }
        // Convert P2 to T2 ^w_T_a
        Eigen::Matrix<T,3,3> w_T_b;
        {
          T cos_t = cos( P2[2] );
          T sin_t = sin( P2[2] );
          w_T_b(0,0) = cos_t;
          w_T_b(0,1) = -sin_t;
          w_T_b(1,0) = sin_t;
          w_T_b(1,1) = cos_t;
          w_T_b(0,2) = P2[0];
          w_T_b(1,2) = P2[1];

          w_T_b(2,0) = T(0.0);
          w_T_b(2,1) = T(0.0);
          w_T_b(2,2) = T(1.0);
        }

        // cast from double to T
        Eigen::Matrix<T, 3, 3> T_a_Tcap_b;
        T_a_Tcap_b <<   T(a_Tcap_b(0,0)), T(a_Tcap_b(0,1)),T(a_Tcap_b(0,2)),
                        T(a_Tcap_b(1,0)), T(a_Tcap_b(1,1)),T(a_Tcap_b(1,2)),
                        T(a_Tcap_b(2,0)), T(a_Tcap_b(2,1)),T(a_Tcap_b(2,2));

        // now we have :: w_T_a, w_T_b and a_Tcap_b
        // compute pose difference
        Eigen::Matrix<T,3,3> diff = T_a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);

        // psi - scalar
        // T psi = T(1.0) / (T(1.0) + exp( T(-2.0)*s[0] )); // zero-penalization factor 0.1
        T psi = max( T(0.0), min( T(1.0), s[0] ) ); //zero-penalization factor 1.5. This needs aggresive zero penalization

        e[0] = psi*diff(0,2);
        e[1] = psi*diff(1,2);
        e[2] = psi*asin( diff(1,0) );
        e[3] = T(1.5) * (this->s_prior - s[0]);

        return true;
    }

    double dx;
    double dy;
    double dtheta;
    double s_prior;
    Eigen::Matrix<double,3,3> a_Tcap_b;

    static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta, const double s_prior_obs){
        return (new ceres::AutoDiffCostFunction<SwitchableClosureResidue, 4, 3, 3, 1>(
            new SwitchableClosureResidue(dx, dy, dtheta, s_prior_obs)));
    };
};

// Dynamic Covariance Scaling
struct DCSClosureResidue
{
    // Observation for the edge
    DCSClosureResidue(double dx, double dy, double dtheta )
    {	
		//double drand48(void);
		//double v = drand48(void);
        this->dx = dx;
        this->dy = dy;
        this->dtheta = dtheta;
        this->s_cap = drand48() * .1 + .9;
        // make a_Tcap_b
        {
          double cos_t = cos( this->dtheta );
          double sin_t = sin( this->dtheta );
          a_Tcap_b(0,0) = cos_t;
          a_Tcap_b(0,1) = -sin_t;
          a_Tcap_b(1,0) = sin_t;
          a_Tcap_b(1,1) = cos_t;
          a_Tcap_b(0,2) = this->dx;
          a_Tcap_b(1,2) = this->dy;

          a_Tcap_b(2,0) = 0.0;
          a_Tcap_b(2,1) = 0.0;
          a_Tcap_b(2,2) = 1.0;
        }
    }

    // Define the residue for each edge. P1 and P2 are 3-vectors representing state of the node ie. x,y,theta
    template <typename T>
    bool operator()(const T* const P1, const T* const P2, T* e) const
    {
        // Convert P1 to T1 ^w_T_a
        Eigen::Matrix<T,3,3> w_T_a;
        {
          T cos_t = T(cos( P1[2] ));
          T sin_t = T(sin( P1[2] ));
          w_T_a(0,0) = cos_t;
          w_T_a(0,1) = -sin_t;
          w_T_a(1,0) = sin_t;
          w_T_a(1,1) = cos_t;
          w_T_a(0,2) = P1[0];
          w_T_a(1,2) = P1[1];

          w_T_a(2,0) = T(0.0);
          w_T_a(2,1) = T(0.0);
          w_T_a(2,2) = T(1.0);
        }

        // Convert P2 to T2 ^w_T_a
        Eigen::Matrix<T,3,3> w_T_b;
        {
          T cos_t = cos( P2[2] );
          T sin_t = sin( P2[2] );
          w_T_b(0,0) = cos_t;
          w_T_b(0,1) = -sin_t;
          w_T_b(1,0) = sin_t;
          w_T_b(1,1) = cos_t;
          w_T_b(0,2) = P2[0];
          w_T_b(1,2) = P2[1];

          w_T_b(2,0) = T(0.0);
          w_T_b(2,1) = T(0.0);
          w_T_b(2,2) = T(1.0);
        }

        // cast from double to T
        Eigen::Matrix<T, 3, 3> T_a_Tcap_b;
        T_a_Tcap_b <<   T(a_Tcap_b(0,0)), T(a_Tcap_b(0,1)),T(a_Tcap_b(0,2)),
                        T(a_Tcap_b(1,0)), T(a_Tcap_b(1,1)),T(a_Tcap_b(1,2)),
                        T(a_Tcap_b(2,0)), T(a_Tcap_b(2,1)),T(a_Tcap_b(2,2));

        // now we have :: w_T_a, w_T_b and a_Tcap_b
        // compute pose difference
        Eigen::Matrix<T,3,3> diff = T_a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);

        // psi - scalar (covariance term. See the paper on DCS for derivation)
        // T psi = T(1.0) / (T(1.0) + exp( T(-2.0)*s[0] ));
        // T psi = max( T(0.0), min( T(1.0), s[0] ) );

        T res = diff(0,2)*diff(0,2) + diff(1,2)*diff(1,2); // + asin( diff(1,0) )*asin( diff(1,0) );
        // T psi_org = T(.3) * T(s_cap) / ( T(1.0) + res ) ;
        T psi_org = sqrt( T(2.0) * T( .5 ) / ( T(.5) + res ) );
        // e[0] = psi ;
        // e[1] = T(0.0);
        // e[2] = T(0.0);
        // return true;
        T psi = min( T(1.0), psi_org ) ;
        e[0] = psi*diff(0,2);
        e[1] = psi*diff(1,2);
        e[2] = psi*asin( diff(1,0) );

        return true;
    }

    double dx;
    double dy;
    double dtheta;
    double s_cap;
    Eigen::Matrix<double,3,3> a_Tcap_b;

    static ceres::CostFunction* Create(const double dx, const double dy, const double dtheta){
        return (new ceres::AutoDiffCostFunction<DCSClosureResidue, 3, 3, 3>(
            new DCSClosureResidue(dx, dy, dtheta)));
    };
};

int main()
{
    //***********************************************//
    // ------------>  Read g2o file  <-------------- //
    //***********************************************//
	std::string BASE_PATH = std::string("..");
     
    string fname = BASE_PATH + "/input_M3500.g2o";
	cout << fname << endl;
    cout << "Start Reading PoseGraph\n";
    ReadG2O g( fname );
    g.add_random_C(25);

    g.writePoseGraph_nodes(BASE_PATH+"/init_nodes.txt");
    g.writePoseGraph_edges(BASE_PATH+"/init_edges.txt");
    cout << "Total nodes : "<< g.nNodes.size() << endl;
    cout << "Total nEdgesOdometry : "<< g.nEdgesOdometry.size() << endl;
    cout << "Total nEdgesClosure : "<< g.nEdgesClosure.size() << endl;
    cout << "Total nEdgesBogus : "<< g.nEdgesBogus.size() << endl;

	//********************************************************//
	// ------------>  Make the cost function  <-------------- //
	//********************************************************//
    ceres::Problem problem;
    ceres::LossFunction * loss_function = NULL;
    loss_function = new ceres::HuberLoss(0.01);

    // Odometry Constraints
    for( int i=0 ; i<g.nEdgesOdometry.size() ; i++ )
    {
        Edge* ed = g.nEdgesOdometry[i];
        ceres::CostFunction * cost_function = OdometryResidue::Create( ed->x, ed->y, ed->theta );

        problem.AddResidualBlock( cost_function, loss_function, ed->a->p, ed->b->p );
    }

    // Loop closure constraints (dynamic covariance scaling)
    for( int i=0 ; i<g.nEdgesClosure.size() ; i++ )
    {
        Edge* ed = g.nEdgesClosure[i];
        ceres::CostFunction * cost_function = DCSClosureResidue::Create( ed->x, ed->y, ed->theta );

        problem.AddResidualBlock( cost_function, loss_function, ed->a->p, ed->b->p );
    }

    for( int i=0 ; i<g.nEdgesBogus.size() ; i++ )
    {
        Edge* ed = g.nEdgesBogus[i];
        ceres::CostFunction * cost_function = DCSClosureResidue::Create( ed->x, ed->y, ed->theta );

        problem.AddResidualBlock( cost_function, loss_function, ed->a->p, ed->b->p );
    }

	//***************************************************//
	// ------------>  Iteratively Solve  <-------------- //
	//***************************************************//

    problem.SetParameterBlockConstant(g.nNodes[0]->p); //1st pose be origin

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;

    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    // options.trust_region_strategy_type = ceres::DOGLEG;
    // options.dogleg_type = ceres::SUBSPACE_DOGLEG;

    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    // options.preconditioner_type = ceres::SCHUR_JACOBI;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout << summary.FullReport() << endl;

    // Write Pose Graph after Optimization
    g.writePoseGraph_nodes(BASE_PATH+"/optimized_nodes.txt");
    g.writePoseGraph_edges(BASE_PATH+"/optimized_edges.txt");
}
