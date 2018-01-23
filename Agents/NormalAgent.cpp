//
//  NormalAgent.cpp
//  Practice
//
//  Created by David Klee on 1/15/18.
//  Copyright © 2018 David Klee. All rights reserved.
//

#include "NormalAgent.hpp"

typedef std::array<double, 2> state_type;

state_type NormalAgent::simulate(state_type& start,double t_step,std::vector<double>& control_scheme,std::vector<state_type >& data_record, std::vector<double>& time_record ) {
    state_type ret;
    ret[0] = start[0]+control_scheme[0];
    ret[1] = start[1]+control_scheme[1];
    data_record.push_back(start);
    data_record.push_back(ret);
    return ret;
}

state_type NormalAgent::simulate(state_type& start,double t_step,std::vector<double>& control_scheme) {
    std::vector<state_type > data_record;
    std::vector<double> time_record;
    return NormalAgent::simulate(start, t_step, control_scheme, data_record, time_record);
}

double NormalAgent::getHeuristicDistance(state_type& state_A,state_type& state_B) {
    return getEuclideanDistance(state_A,state_B);
}

void NormalAgent::getController(const state_type& q_start, const state_type& q_end, const double t_step, state_type& q_new, std::vector<double>& controlScheme) {
    std::array<double,2> vec = {q_end[0]-q_start[0],q_end[1]-q_start[1]};
    double angle = getAngleFromVector(vec);
    controlScheme.clear();
    controlScheme.push_back(t_step*cos(angle));
    controlScheme.push_back(t_step*sin(angle));
}

void NormalAgent::writePath(std::vector<RRT_node<state_type> >& nodes, double step_size) {
    static std::fstream outfile("normal_path.txt", std::fstream::out);
    
    std::vector<RRT_node<state_type> > solution = {nodes.back()};
    RRT_node<state_type> node = nodes.back();
    while (node.parent_id >= 0) {
        solution.push_back(node);
        node = nodes[node.parent_id];
    }
    solution.push_back(nodes[0]);
    
    typedef std::vector<RRT_node<state_type> >::reverse_iterator iter;
    for (iter i=solution.rbegin(); i!= solution.rend();++i) {
        outfile << "(" << i->node_state[0] << "," << i->node_state[1] << ")\n";
    }
}

void NormalAgent::writeTree(std::vector<RRT_node<state_type> >& nodes) {
    std::fstream outfile("normal_tree.txt", std::fstream::out);
    
    size_t num_nodes = nodes.size();
    for (size_t i=1; i != num_nodes; ++i) {
        state_type start = nodes[i].path[0];
        state_type end = nodes[i].path[1];
        outfile << "(" << start[0] << "," << start[1] << "," << end[0] << "," << end[1] << ")\n";
    }
    outfile.close();
}

void NormalAgent::queryStateSpace(state_type& q_rand) {
    q_rand[0] = randDouble(-10.0, 10.0);
    q_rand[1] = randDouble(-10.0, 10.0);
}



