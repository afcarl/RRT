//
//  CarAgent.cpp
//  Practice
//
//  Created by David Klee on 12/7/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#include "CarAgent.hpp"

typedef std::array<double,3> state_type;

std::array<double,3> CarAgent::simulate(const state_type& start,double t_span,
                                        std::vector<double>& controlScheme,
                                        std::vector<state_type >& data_record,
                                        std::vector<double>& time_record) {
    
    state_type current_state;
    std::copy(start.begin(),start.end(),current_state.begin());
    
    double u_speed = controlScheme[0];
    double u_turn = controlScheme[1];
    
    int num_steps =  static_cast<int>(t_span/m_step_size);
    
    for (int i=0; i != num_steps;++i) {
        time_record.push_back(i*m_step_size);
        state_type delta = {u_speed*cos(current_state[2]),
            u_speed*sin(current_state[2]),u_speed*tan(u_turn)/car_length};
        current_state[0] += m_step_size*delta[0];
        current_state[1] += m_step_size*delta[1];
        current_state[2] += m_step_size*delta[2];
        data_record.push_back(current_state);
    }
    return current_state;
}

std::array<double,3> CarAgent::simulate(const state_type& start,double t_span,
                                        std::vector<double>& controlScheme) {
    std::vector<state_type> data_record;
    std::vector<double> time_record;
    return this->simulate(start, t_span, controlScheme, data_record, time_record);
}

double CarAgent::getHeuristicDistance(const state_type& state_A, const state_type& state_B) {
    double ret = 0;
    ret += getEuclideanDistance(state_A, state_B);
    double angle_between = fmod(abs(fmod(state_A[2],2*M_PI)- fmod(state_A[2],2*M_PI)),M_PI);
    if (angle_between > M_PI/2) {
        angle_between = M_PI/2-angle_between;
    }
    double adj_factor;
    if (angle_between < M_PI_4) {
        adj_factor = 1+angle_between/10.0;
    } else {
        adj_factor = 1+pow(angle_between,5.0);
    }
    ret *= adj_factor;
    return ret;
}

void CarAgent::getController(const state_type& q_start, const std::array<double,3>& q_end,
                             const double t_span, state_type& q_new, std::vector<double>& controlScheme) {
    int num_bins = 10;
    int best_bin[] {0,0};
    double best_dist = 100000;
    for (int i = 0; i != 2; ++i) {                          // forward or backward
        for (int j = -num_bins/2; j != num_bins/2+1; ++j) {
            double u_speed = (i==1) ? max_speed : -max_speed;
            double u_turn = max_turn_angle*(2.0*j/static_cast<double>(num_bins));
            std::vector<double> control = {u_speed,u_turn};
            state_type q_probe = this->simulate(q_start,t_span,control);
            double dist = CarAgent::getHeuristicDistance(q_probe, q_end);
            if (dist < best_dist) {
                best_dist = dist;
                best_bin[0] = i;
                best_bin[1] = j;
            }
        };
    };
    controlScheme.clear();
    controlScheme.push_back((best_bin[0]==1) ? max_speed : -max_speed);
    controlScheme.push_back(max_turn_angle*(2.0*best_bin[1]/static_cast<double>(num_bins)));
    q_new = this->simulate(q_start,t_span,controlScheme);
}

void CarAgent::writePath(std::vector<RRT_node<state_type> >& nodes, double step_size){
    std::vector<RRT_node<state_type> > solution;
    RRT_node<state_type> node = nodes.back();
    while (node.parent_id >= 0) {
        solution.push_back(node);
        node = nodes[node.parent_id];
    }
    
    std::vector<state_type> data_record;
    std::vector<double> time_record;
    typedef std::vector<RRT_node<state_type> >::reverse_iterator iter;
    for (iter i=solution.rbegin(); i!= solution.rend();++i) {
        std::copy(i->path.begin(),i->path.end(),back_inserter(data_record));
    }
    
    std::fstream outfile("car_path.txt", std::fstream::out);
    size_t data_size = data_record.size();
    double t_step = m_step_size;
    for (size_t i=0; i!= data_size; ++i) {
        outfile << "(" << i*t_step << "," << data_record[i][0] << ",";
        outfile << data_record[i][1] << "," << data_record[i][2] << ")\n";
    };
}

void CarAgent::writeTree(std::vector<RRT_node<state_type> >& nodes) {
    std::fstream outfile("car_tree.txt", std::fstream::out);
    
    size_t num_nodes = nodes.size();
    for (size_t i=1; i != num_nodes; ++i) {
        std::vector<state_type> path = nodes[i].path;
        size_t path_length = path.size();
        outfile << "(";
        for (size_t j=0; j != path_length; ++j) {
            outfile << path[j][0] << "," << path[j][1] << "," << path[j][2] << ",";
        }
        outfile << ")\n";
    }
    outfile.close();
}

void CarAgent::queryStateSpace(state_type& q_rand) {
    q_rand[0] = randDouble(-10.0, 10.0);
    q_rand[1] = randDouble(-10.0, 10.0);
    q_rand[2] = randDouble(0.0, 2*M_PI);
}
