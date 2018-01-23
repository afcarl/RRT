//
//  Path.hpp
//  Practice
//
//  Created by David Klee on 12/7/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#ifndef Path_hpp
#define Path_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include "HelperFunctions.hpp"

class Path{
public:
    void initialize(std::vector<xy_type>& starting_path);
    void initialize();
    double getMovementScore(std::vector<xy_type>& movement,int start_ID);
    int getClosestPointID(xy_type& target_point);
    void writeToFile();
private:
    std::vector<xy_type> m_path;
    int direction = 1; // positive for clockwise
    xy_type m_center;
};




#endif /* Path_hpp */
