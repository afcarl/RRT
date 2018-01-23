//
//  PathFinder.hpp
//  Practice
//
//  Created by David Klee on 12/7/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#ifndef PathTracer_hpp
#define PathTracer_hpp

#include <stdio.h>
#include <vector>
#include "HelperFunctions.hpp"
#include "Path.hpp"

typedef std::array<double,3> state_type;
struct Movement {
    boost::array<double,2> controlScheme; // {tau1,tau2}
    std::vector<xy_type> path;
};

class PathTracer
{
public:
    void initialize(state_type& start_state);
    void scanControls(int beam_size, Movement& best_movement);
    void extend();
private:
    Path m_path;
    std::vector<xy_type> m_trace;
};

#endif /* PathTracer_hpp */
