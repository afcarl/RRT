//
//  LegAgent.hpp
//  Practice
//
//  Created by David Klee on 1/12/18.
//  Copyright © 2018 David Klee. All rights reserved.
//

#ifndef LegAgent_hpp
#define LegAgent_hpp

#include <stdio.h>
#include <iostream>
#include <array>
#include "RRTClass.hpp"
#include <fstream>
#include <boost/numeric/odeint.hpp>
#include "HelperFunctions.hpp"


//[ ode_wrapper
template< class Obj , class Mem >
class ode_wrapper
{
    Obj m_obj;
    Mem m_mem;
    
public:
    
    ode_wrapper( Obj obj , Mem mem ) : m_obj( obj ) , m_mem( mem ) { }
    
    template< class State , class Deriv , class Time >
    void operator()( const State &x , Deriv &dxdt , Time t )
    {
        (m_obj.*m_mem)( x , dxdt , t );
    }
};

template< class Obj , class Mem >
ode_wrapper< Obj , Mem > make_ode_wrapper( Obj obj , Mem mem )
{
    return ode_wrapper< Obj , Mem >( obj , mem );
}
//]


template< class Obj , class Mem >
class observer_wrapper
{
    Obj m_obj;
    Mem m_mem;
    
public:
    observer_wrapper( Obj obj , Mem mem ) : m_obj( obj ) , m_mem( mem ) { }
    
    template< class State , class Time >
    void operator()( const State &x , Time t )
    {
        (m_obj.*m_mem)( x , t );
    }
};

template< class Obj , class Mem >
observer_wrapper< Obj , Mem > make_observer_wrapper( Obj obj , Mem mem )
{
    return observer_wrapper< Obj , Mem >( obj , mem );
}



//[ bind_member_function
class LegAgent
{
private:
    // Physical parameters of system
    const double max_deltaU = 100.0;           // max change in torque per second
    const double max_Tau1 = 10.0;
    const double max_Tau2 = 10.0;
    const double G = 9.81;
    const double L1 = 1.0;
    const double L2 = 1.0;
    const double M1 = 1.0;
    const double M2 = 1.0;
    const double B1 = 0.5;                 // damping constant at hip
    const double B2 = 0.3;                 //        ''        at knee
    double Tau1,Tau2;
    boost::numeric::ublas::matrix<double> M_term;
    boost::numeric::ublas::matrix<double> M_inv;
    boost::numeric::ublas::vector< double > b_term;
    boost::numeric::ublas::vector< double > x_term;
    
    std::vector<std::array<double,4> >* p_data_record;
    std::vector<double>* p_time_record;
    
    void ode( const std::array<double,4>& x , std::array<double,4>& dxdt , double t );
    void obs( const std::array<double,4>& x , double t );
    bool invert_matrix (const boost::numeric::ublas::matrix<double>& input, boost::numeric::ublas::matrix<double>& inverse);
    void initialize();
public:
    LegAgent();
    LegAgent(double tau1, double tau2, std::vector<std::array<double,4> >& p_data,std::vector<double>& time_record);
    std::array<double,4> simulate(const std::array<double,4>& start,double t_span,std::vector<double>& controlScheme,
                        std::vector<std::array<double,4> >& data_record,std::vector<double>& time_record);
    std::array<double,4> simulate(const std::array<double,4>& start,double t_span, std::vector<double>& controlScheme);
    xy_type convertStateToXY(const std::array<double,4>& state);
    std::vector<xy_type> convertStateToXY(const std::vector<std::array<double,4> >& state_path);
    double getHeuristicDistance(const std::array<double,4>& state_A, const std::array<double,4>& state_B);
    void getController(const std::array<double,4>& q_start, const std::array<double,4>& q_end, const double t_span, std::array<double,4>& q_new, std::vector<double>& controlScheme);
    void writePath(std::vector<RRT_node<std::array<double,4> > >& nodes, double step_size);
    void writeTree(std::vector<RRT_node<std::array<double,4> > >& nodes);
    void queryStateSpace(std::array<double,4>& q_rand);
};




#endif /* LegAgent_hpp */
