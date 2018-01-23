//
//  LegAgent.cpp
//  Practice
//
//  Created by David Klee on 1/12/18.
//  Copyright © 2018 David Klee. All rights reserved.
//

#include "LegAgent.hpp"

typedef std::array<double,4> state_type;

LegAgent::LegAgent(): M_term(2,2),M_inv(2,2),b_term(2),x_term(2),Tau1(0),Tau2(0) {
    LegAgent::initialize();
}

LegAgent::LegAgent(double tau1, double tau2,
                   std::vector<state_type>& data_record,std::vector<double>& time_record):
                   M_term(2,2),M_inv(2,2),b_term(2),x_term(2),Tau1(tau1),Tau2(tau2) {
                       LegAgent::initialize();
    p_data_record = &data_record;
    p_time_record = &time_record;
}

void LegAgent::initialize() {
    M_term (0,0) = (M1+M2)*L1;
    M_term (1,1) = L2;
}

bool LegAgent::invert_matrix (const boost::numeric::ublas::matrix<double>& input, boost::numeric::ublas::matrix<double>& inverse) {
    typedef boost::numeric::ublas::permutation_matrix<std::size_t> pmatrix;
    
    // create a working copy of the input
    boost::numeric::ublas::matrix<double> A(input);
    
    // create a permutation matrix for the LU-factorization
    boost::numeric::ublas::permutation_matrix<double> pm(A.size1());
    
    // perform LU-factorization
    int res = static_cast<int> (boost::numeric::ublas::lu_factorize(A, pm));
    if (res != 0)
        return false;
    
    // create identity matrix of "inverse"
    inverse.assign(boost::numeric::ublas::identity_matrix<double> (A.size1()));
    
    // backsubstitute to get the inverse
    boost::numeric::ublas::lu_substitute(A, pm, inverse);
    
    return true;
}

void LegAgent::ode( const state_type &x , state_type &dxdt , double t )
{
    M_term (0,1) = M2*L2*cos(x[1]-x[0]);
    M_term (1,0) = L1*cos(x[1]-x[0]);
    b_term (0) = Tau1-B1*x[2]+M2*L2*pow(x[3],2.0)*sin(x[1]-x[0])-(M1+M2)*G*sin(x[0]);
    b_term (1) = Tau2-B2*x[3]-L1*pow(x[2],2.0)*sin(x[1]-x[0])-G*sin(x[1]);
    LegAgent::invert_matrix(M_term, M_inv);
    x_term = boost::numeric::ublas::prod(M_inv, b_term);
    dxdt[0] = x[2];
    dxdt[1] = x[3];
    dxdt[2] = x_term(0);
    dxdt[3] = x_term(1);
}

void LegAgent::obs( const state_type &x , double t )
{
//    std::cout << t << " " << x[0] << " " << x[1] << "\n";
    p_data_record->push_back(x);
    p_time_record->push_back(t);
}

state_type LegAgent::simulate(const state_type& start,double t_span, std::vector<double>& controlScheme,
                            std::vector<state_type>& data_record,std::vector<double>& time_record) {
    p_data_record = &data_record;
    p_time_record = &time_record;
    
    // create dummy array for q_start because it gets written
    state_type start_temp;
    std::copy(start.begin(), start.end(), start_temp.begin());
    
    // get t_step
    const double num_steps = 20;
    const double lbound_step = 0.001;           // lower bound on the step size
    double t_step = t_span/num_steps;
    t_step = t_step > lbound_step ? t_step : lbound_step;
    
    double tau1 = controlScheme[0];
    double tau2 = controlScheme[1];
    
    using namespace boost::numeric::odeint;
    integrate_const(runge_kutta4< state_type >(),make_ode_wrapper( LegAgent(tau1, tau2, data_record,time_record) , &LegAgent::ode ) ,
              start_temp , 0.0 , t_span , t_step , make_observer_wrapper( LegAgent(tau1, tau2, data_record,time_record) , &LegAgent::obs ) );
    
    
//    std::cout<< "Given: (" << start[0] << "," << start[1] << ")\n";
//    std::cout<< "Front: (" << data_record.front()[0] << "," << data_record.front()[1] << ")\n";
    return data_record.back();
}

state_type LegAgent::simulate(const state_type& start,double t_span, std::vector<double>& controlScheme) {
    std::vector<state_type> data_record;
    std::vector<double> time_record;
    return this->simulate(start, t_span, controlScheme, data_record, time_record);
}

xy_type LegAgent::convertStateToXY(const state_type& state) {
    xy_type point_xy = {L1*sin(state[0])+L2*sin(state[1]),
        -L1*cos(state[0])-L2*cos(state[1])};
    return point_xy;
}

std::vector<xy_type> LegAgent::convertStateToXY(const std::vector<state_type>& state_path) {
    std::vector<xy_type> xy_path;
    for (int i=0; i<state_path.size();++i) {
        xy_path.push_back(LegAgent::convertStateToXY(state_path[i]));
    }
    return xy_path;
}

double LegAgent::getHeuristicDistance(const state_type& state_A, const state_type& state_B) {
    xy_type point_A = this->convertStateToXY(state_A);
    xy_type point_B = this->convertStateToXY(state_B);
    return getEuclideanDistance(point_A,point_B);
}

void LegAgent::getController(const state_type& q_start, const state_type& q_end, const double t_span, state_type& q_new, std::vector<double>& controlScheme) {
    int num_bins = 6;
    int best_bin[] {0,0};
    double best_dist = 100000;
    for (int i = -num_bins/2; i != num_bins/2; ++i) {
        for (int j = -num_bins/2; j != num_bins/2; ++j) {
            double tau1 = max_Tau1*(2.0*i/static_cast<double>(num_bins));
            double tau2 = max_Tau2*(2.0*j/static_cast<double>(num_bins));
            std::vector<double> control = {tau1,tau2};
            state_type q_probe = this->simulate(q_start,t_span,control);
            double dist = LegAgent::getHeuristicDistance(q_probe, q_end);
            if (dist < best_dist) {
                best_dist = dist;
                best_bin[0] = i;
                best_bin[1] = j;
            }
        };
    };
    controlScheme.clear();
    controlScheme.push_back(max_Tau1*(1.0-best_bin[0]/static_cast<double>(num_bins)));
    controlScheme.push_back(max_Tau2*(1.0-best_bin[1]/static_cast<double>(num_bins)));
    q_new = this->simulate(q_start,t_span,controlScheme);
}

void LegAgent::writePath(std::vector<RRT_node<state_type> >& nodes,double step_size) {
    std::vector<state_type> data_record;
    std::vector<double> time_record;
    RRT_node<state_type> node = nodes.back();
    while (node.parent_id > 0) {
        RRT_node<state_type> parent = nodes[node.parent_id];
        state_type start = parent.node_state;
        this->simulate(start, step_size, node.control_scheme, data_record, time_record);
        node = parent;
    }
    
    std::fstream outfile("leg_path.txt", std::fstream::out);
    size_t data_size = data_record.size();
    double t_step = time_record[0]-time_record[1];
    for (size_t i=0; i!= data_size; ++i) {
        outfile << "(" << i*t_step << "," << data_record[i][0] << "," << data_record[i][1] << ")\n";
    };
}

void LegAgent::writeTree(std::vector<RRT_node<state_type> >& nodes) {
    size_t num_nodes = nodes.size();
    std::fstream outfile("leg_tree.txt",std::fstream::out);
    for (size_t i=0; i != num_nodes; ++i) {
        state_type state = nodes[i].node_state;
        outfile << "(" << state[0] << "," << state[1] << ")\n";
    }
    outfile.close();
}

void LegAgent::queryStateSpace(state_type& q_rand ) {
    q_rand[0] = randDouble(0.0,2*M_PI);
    q_rand[1] = randDouble(0.0,2*M_PI);
    q_rand[2] = 0.0;
    q_rand[3] = 0.0;
}

