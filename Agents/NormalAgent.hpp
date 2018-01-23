//
//  NormalAgent.hpp
//  Practice
//
//  Created by David Klee on 1/15/18.
//  Copyright © 2018 David Klee. All rights reserved.
//

#ifndef NormalAgent_hpp
#define NormalAgent_hpp

#include <stdio.h>
#include <array>
#include <iostream>
#include "HelperFunctions.hpp"
#include "RRTClass.hpp"

class NormalAgent
{
public:
    std::array<double,2> simulate(std::array<double,2>& start_state,double t_step,std::vector<double>& control_scheme,std::vector<std::array<double,2> >& data_record, std::vector<double>& time_record );
    std::array<double,2> simulate(std::array<double,2>& start_state,double t_step,std::vector<double>& control_scheme);

    double getHeuristicDistance(std::array<double,2>& state_A,std::array<double,2>& state_B);

    void getController(const std::array<double,2>& q_start, const std::array<double,2>& q_end, const double t_step, std::array<double,2>& q_new, std::vector<double>& controlScheme);

    void writePath(std::vector<RRT_node<std::array<double,2> > >& nodes, double step_size);
    
    void writeTree(std::vector<RRT_node<std::array<double,2> > >& nodes);
    
    void queryStateSpace(std::array<double,2>& q_rand);
    
};









#endif /* NormalAgent_hpp */
