//
//  Environment.cpp
//  Practice
//
//  Created by David Klee on 12/7/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#include "FlatWorld.hpp"

void FlatWorld::initialize() {
    xy_type pointA = {0.0,0.0};
    xy_type pointB = {4.0,0.0};
    xy_type pointC = {0.0,4.0};
    FlatWorld::add_obstacle(pointA, pointB, pointC);
    
    xy_type pointD = {-1.0,-3.0};
    xy_type pointE = {2.0,-3.4};
    xy_type pointF = {0.0,0.0};
    FlatWorld::add_obstacle(pointD, pointE, pointF);
}

void FlatWorld::add_obstacle(const std::array<double,2>& pointA, const std::array<double,2>& pointB, const std::array<double,2>& pointC) {
    std::array<double,3> eqn1 = {(pointA[1]-pointB[1]), (pointB[0]-pointA[0]), (pointA[0]*pointB[1]-pointB[0]*pointA[1])};
    if (not FlatWorld::evaluate_equation(eqn1, pointC)) {
        eqn1[0] *= -1.0;
        eqn1[1] *= -1.0;
        eqn1[2] *= -1.0;
    }
    std::array<double,3> eqn2 = {(pointB[1]-pointC[1]), (pointC[0]-pointB[0]), (pointB[0]*pointC[1]-pointC[0]*pointB[1])};
    if (not FlatWorld::evaluate_equation(eqn2, pointA)) {
        eqn2[0] *= -1.0;eqn2[1] *= -1.0;eqn2[2] *= -1.0;
    }
    std::array<double,3> eqn3 = {(pointC[1]-pointA[1]), (pointA[0]-pointC[0]), (pointC[0]*pointA[1]-pointA[0]*pointC[1])};
    if (not FlatWorld::evaluate_equation(eqn3, pointB)) {
        eqn3[0] *= -1.0;eqn3[1] *= -1.0;eqn3[2] *= -1.0;
    }
    Obstacle new_obs = new Obstacle(eqn1,eqn2,eqn3);
    FlatWorld::m_all_obstacles.push_back(new_obs);
}

bool FlatWorld::evaluate_equation(const std::array<double,3>& eqn, const std::array<double, 2>& point) {
    return (point[0]*eqn[0]+point[1]*eqn[1] + eqn[2] > 0.0);
}

bool FlatWorld::check_within_bounds(const std::array<double,2>& point) {
    // not sure how this applies for walking
    return true;
}

bool FlatWorld::check_obstacle(const Obstacle& obs, const std::array<double,2>& point) {
    std::array<double,2> temp_point = {point[0],point[1]};
    if (FlatWorld::evaluate_equation(obs.eqn1, temp_point) and
        FlatWorld::evaluate_equation(obs.eqn2, temp_point) and
        FlatWorld::evaluate_equation(obs.eqn3, temp_point)) {
        return true;
    }
    return false;
}

bool FlatWorld::check_if_free(const std::array<double,2>& point) {
    bool free = FlatWorld::check_within_bounds(point);
    if (free) {
        int num_obs = static_cast<int>(FlatWorld::m_all_obstacles.size());
        for (int i=0; i< num_obs;++i) {
            free = check_obstacle(FlatWorld::m_all_obstacles[i], point);
            if (not free) {
                break;
            }
        }
    }
    return free;
}

void delete_obstacles() {
    
}


void FlatWorld::writeToFile()
{
    std::fstream outfile("Environment.txt", std::fstream::out);
    int num_obs = static_cast<int>(Environment::m_all_obstacles.size());
    for (int i=0;i<num_obs;++i) {
        outfile << "(" << Environment::m_all_obstacles[i].eqn1[0] << "," << Environment::m_all_obstacles[i].eqn1[1] << "," <<
            Environment::m_all_obstacles[i].eqn1[2] << "):";
        outfile << "(" << Environment::m_all_obstacles[i].eqn2[0] << "," << Environment::m_all_obstacles[i].eqn2[1] << "," <<
            Environment::m_all_obstacles[i].eqn2[2] << "):";
        outfile << "(" << Environment::m_all_obstacles[i].eqn3[0] << "," << Environment::m_all_obstacles[i].eqn3[1] << "," <<
            Environment::m_all_obstacles[i].eqn3[2] << ")\n";
    }
    outfile.close();
}

