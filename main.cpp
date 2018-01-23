//
//  main.cpp
//  Practice
//
//  Created by David Klee on 9/29/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#include <iostream>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <array>
#include <math.h>
#include <random>
#include <fstream> //for copying to files
#include "HelperFunctions.hpp"
#include "LegAgent.hpp"
#include "CarAgent.hpp"
#include "NormalAgent.hpp"
#include "RRTClass.hpp"
#include <boost/numeric/ublas/matrix.hpp>


///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

/* Task list:
    - add collision detection to RRT
    - create CarAgent
 */


int main( int argc , char *argv[] )
{
//    typedef std::array<double,3> state_type;
//    state_type start = {{-9.0,-9.0,0.0}};
//    state_type goal = {{2.0,4.0,0.0}};
//    RRT<state_type,CarAgent> tree(start,goal,0.5);
    
    typedef std::array<double,2> state_type;
    state_type start = {{0.0,0.0}};
    state_type goal = {{2.0,2.0}};
    double branch_length = 0.1;
    RRT<state_type,NormalAgent> tree(start,goal,branch_length);
    tree.setGoalBias(0.2);

//    tree.setGoalBias(0.2);
    tree.build_RRT(1000);
}




