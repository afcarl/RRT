//
//  Path.cpp
//  Practice
//
//  Created by David Klee on 12/7/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#include "Path.hpp"

void Path::initialize(std::vector<xy_type>& starting_path)
{
        Path::m_path = starting_path;
}

void Path::initialize()
{
    double angle;
    for (int i=0; i < 3600; ++i) {
        angle = static_cast<double>(i)*M_PI/180.0;
        xy_type new_point = {0.5*cos(angle),0.5*sin(angle)-1.5};
        Path::m_path.push_back(new_point);
    }
    m_center = {0,-1.5};
}

double Path::getMovementScore(std::vector<xy_type>& movement,int start_ID)
{
    double score = 0;
    return score;
}

int Path::getClosestPointID(xy_type& target_point)
{
    double shortest_distance = 1000;
    int best_ID = 0;
    for (int i=0;i<Path::m_path.size();++i) {
        double dist = getEuclideanDistance(Path::m_path[i], target_point);
        if (dist < shortest_distance) {
            shortest_distance = dist;
            best_ID = i;
        }
    }
    return best_ID;
}

void Path::writeToFile()
{
    std::fstream outfile("path.txt", std::fstream::out);
    int num_points = static_cast<int>(m_path.size());
    for (int i=0; i < num_points; ++i) {
        outfile << "(" << m_path[i][0] << "," << m_path[i][1] << ")\n";
    }
    outfile.close();
}
