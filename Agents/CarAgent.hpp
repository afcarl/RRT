//
//  CarAgent.hpp
//  Practice
//
//  Created by David Klee on 12/7/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#ifndef CarAgent_hpp
#define CarAgent_hpp

#include <stdio.h>
#include <array>
#include <vector>
#include <fstream>
#include <iostream>
#include <cmath>
#include "HelperFunctions.hpp"
#include "RRTClass.hpp"

class CarAgent
{
private:
    // Physical parameters of system
    const double car_length = 0.5;
    const double min_turn_radius = 1.0;  // put it about 3-4 times the car_length
    const double max_turn_angle = atan(car_length/min_turn_radius);
    const double max_speed = 1.0;
    
    double m_step_size = 0.02;                 // in units time

public:
    std::array<double,3> simulate(const std::array<double,3>& start,double t_span,std::vector<double>& controlScheme,
                                  std::vector<std::array<double,3> >& data_record,std::vector<double>& time_record);
    std::array<double,3> simulate(const std::array<double,3>& start,double t_span, std::vector<double>& controlScheme);
    xy_type convertStateToXY(const std::array<double,3>& state);
    std::vector<xy_type> convertStateToXY(const std::vector<std::array<double,3> >& state_path);
    double getHeuristicDistance(const std::array<double,3>& state_A, const std::array<double,3>& state_B);
    void getController(const std::array<double,3>& q_start, const std::array<double,3>& q_end, const double t_span, std::array<double,3>& q_new, std::vector<double>& controlScheme);
    void writePath(std::vector<RRT_node<std::array<double,3> > >& nodes, double step_size);
    void writeTree(std::vector<RRT_node<std::array<double,3> > >& nodes);
    void queryStateSpace(std::array<double,3>& q_rand);
};



#endif /* CarAgent_hpp */


