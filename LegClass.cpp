//
//  LegClass.cpp
//  Practice
//
//  Created by David Klee on 11/30/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#include "LegClass.hpp"

Leg::Leg(): M_term(2,2),M_inv(2,2),b_term(2),x_term(2),Tau1(0),Tau2(0) {
    initialize();
}

Leg::Leg(double t1,double t2): M_term(2,2),M_inv(2,2),b_term(2),x_term(2),Tau1(t1),Tau2(t2) {
    initialize();
}

void Leg::initialize() {
    M_term (0,0) = (M1+M2)*L1;
    M_term (1,1) = L2;
}

void Leg::dynamics( const state_type &x , state_type &dxdt , double t ) {
    M_term (0,1) = M2*Leg::L2*cos(x[1]-x[0]);
    M_term (1,0) = L1*cos(x[1]-x[0]);
    b_term (0) = Tau1-B1*x[2]+M2*L2*pow(x[3],2.0)*sin(x[1]-x[0])-(M1+M2)*G*sin(x[0]);
    b_term (1) = Tau2-B2*x[3]-L1*pow(x[2],2.0)*sin(x[1]-x[0])-G*sin(x[1]);
    Leg::InvertMatrix(M_term, M_inv);
    x_term = boost::numeric::ublas::prod(M_inv, b_term);
    dxdt[0] = x[2];
    dxdt[1] = x[3];
    dxdt[2] = x_term(0);
    dxdt[3] = x_term(1);
}

bool Leg::InvertMatrix (const boost::numeric::ublas::matrix<double>& input, boost::numeric::ublas::matrix<double>& inverse) {
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

void Leg::write_data(const state_type &x , double t )
{
//    std::cout << t << " (" << x[0] << "," << x[1] << ") \n";
    static std::fstream outfile("leg.txt", std::fstream::out);
    outfile << "(" << t << "," << x[0] << "," << x[1] << ") \n";
}

void Leg::simulate(const state_type& start, const double t_span, double tau1, double tau2) {
    data_record.clear();
    time_record.clear();
    Tau1 = tau1;
    Tau2 = tau2;
    double t_step = 0.00001;
    // std::bind( &Leg::write_data,Leg(tau1,tau2), std::placeholders::_1 , std::placeholders::_2)
//    boost::numeric::odeint::integrate(dynamics, start, 0.0, t_span, t_step)
//    m_outfile.close();
}


double Leg::getEnergy(state_type& state)
{
    // potential energy, defined to be always >= 0
    double V = L1+L2+(-L1*cos(state[0]))*M1*G + (-L1*cos(state[0])-L2*cos(state[1]))*M2*G;
    // kinetic energy, always >= 0
    double T = 0.5*M1*pow(L1*state[2],2.0) + 0.5*M2*(pow(L1*state[2],2.0)+pow(L2*state[3],2.0)+
                                                     2*L1*L2*state[2]*state[3]*cos(state[0]-state[1]));
    return V+T;
}

xy_type Leg::convertStateToXY(const state_type& state) {
    xy_type point_xy = {L1*sin(state[0])+L2*sin(state[1]),
        -L1*cos(state[0])-L2*cos(state[1])};
    return point_xy;
}

std::vector<xy_type> Leg::convertStateToXY(const std::vector<state_type>& state_path) {
    std::vector<xy_type> xy_path;
    for (int i=0; i<state_path.size();++i) {
        xy_path.push_back(Leg::convertStateToXY(state_path[i]));
    }
    return xy_path;
}

double Leg::getHeuristicDistance(const state_type& state_A, const state_type& state_B) {
    double dist = 0;
    dist += (pow(fmod(state_A[0]-state_B[0],2.0*M_PI),2.0) + pow(fmod(state_A[1]-state_B[1],2.0*M_PI),2.0));
    dist += (pow(state_A[2]-state_B[2],2.0)+pow(state_A[2]-state_B[2],2.0))/1000.0;
    return dist;
}

void Leg::getController(const state_type& q_start, const state_type& q_end, const double t_step, state_type& q_new, std::vector<double>& controlScheme) {
    std::cout << "need to add a function body for Leg::getController" << "\n";
}

