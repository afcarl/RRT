//
//  LegClass.hpp
//  Practice
//
//  Created by David Klee on 11/30/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#ifndef LegClass_hpp
#define LegClass_hpp

#include <stdio.h>
#include <vector>
#include <cmath>
#include <fstream>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "HelperFunctions.hpp"

typedef std::array<double,4> state_type;

void ode( const state_type &x , state_type &dxdt , double t );
void callIntegrate(state_type& start,double t_span,double t_step);

class Leg
{
private:
    const double max_deltaU = 100.0;           // max change in torque per second
    const double max_Tau1 = 5.0;
    const double max_Tau2 = 5.0;
    const double G = 9.81;
    const double L1 = 1.0;
    const double L2 = 1.0;
    const double M1 = 1.0;
    const double M2 = 1.0;
    const double B1 = 0.5;                 // damping constant at hip
    const double B2 = 0.3;                 //        ''        at knee
    double Tau1,Tau2;
    std::vector< state_type> data_record;
    std::vector< double > time_record;
    boost::numeric::ublas::matrix<double> M_term;
    boost::numeric::ublas::matrix<double> M_inv;
    boost::numeric::ublas::vector< double > b_term;
    boost::numeric::ublas::vector< double > x_term;
public:
    Leg();
    Leg(double,double);
    void initialize();
    void dynamics( const state_type& x , state_type& dxdt , double t );
    void write_data(const state_type &x , const double t );
    void simulate(const state_type& start, const double t_span,double tau1,double tau2);
    bool InvertMatrix (const boost::numeric::ublas::matrix<double>& input, boost::numeric::ublas::matrix<double>& inverse);
    double getEnergy(state_type& state);
    xy_type convertStateToXY(const state_type& state);
    std::vector<xy_type> convertStateToXY(const std::vector<state_type>& state_path);
    double getHeuristicDistance(const state_type& state_A, const state_type& state_B);
    void getController(const state_type& q_start, const state_type& q_end, const double t_step, state_type& q_new, std::vector<double>& controlScheme);
};

#endif /* LegClass_hpp */
