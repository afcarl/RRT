//
//  Environment.hpp
//  Practice
//
//  Created by David Klee on 12/7/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#ifndef FlatWorld_hpp
#define FlatWorld_hpp

#include <stdio.h>
#include <array>
#include "HelperFunctions.hpp"
#include <iostream>
#include <vector>
#include <fstream>

struct Obstacle
{
    Obstacle(std::array<double,3>& EQ1,std::array<double,3>& EQ2,std::array<double,3>& EQ3) {
        std::copy(EQ1.begin(),EQ1.end(),eqn1.begin());
        std::copy(EQ2.begin(),EQ2.end(),eqn2.begin());
        std::copy(EQ3.begin(),EQ3.end(),eqn3.begin());
    }
    std::array<double,3> eqn1;  //ax+by+c > 0
    std::array<double,3> eqn2;
    std::array<double,3> eqn3;
};

class FlatWorld
{
private:
    void initialize();
    void delete_obstacles();
    bool check_within_bounds(const std::array<double,2>& point);
    bool evaluate_equation(const std::array<double,3>& eqn, const std::array<double,2>& point);
    bool check_obstacle(const Obstacle& obs, const std::array<double,2>& point);
    std::vector<Obstacle> m_all_obstacles;
    std::vector<std::array<double,2> > m_bounds;  // EACH ARRAY HOLDS UPPER AND LOWER BOUNDS FOR EACH STATE VARIABLE
public:
    FlatWorld() {
        initialize();
    }
    ~FlatWorld() {
        delete_obstacles();
    }
    void setBounds(const std::vector<std::array<double,2> >& bounds);
    std::vector<std::array<double,2> > getBounds();
    void add_obstacle(const std::array<double,2>& pointA, const std::array<double,2>& pointB, const std::array<double,2>& pointC);
    bool check_if_free(const std::array<double,2>& point);
    void writeToFile();
};


#endif /* FlatWorld_hpp */
