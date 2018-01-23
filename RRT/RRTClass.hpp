//
//  RRTClass.hpp
//  Practice
//
//  Created by David Klee on 12/6/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#ifndef RRTClass_hpp
#define RRTClass_hpp

#include <stdio.h>
#include <map>
#include "HelperFunctions.hpp"
#include "Environment.hpp"

template <class State>
struct RRT_node
// specific to each agent
{
    State node_state;
    std::vector<double> control_scheme;
    std::vector<State> path;
    int parent_id;
};

template <class State,class Agent>
class RRT
{
public:
    RRT() {
        std::cout << "You must set the start and goal." << "\n";
    }
    
    RRT(const State& start, const State& goal, const double step_size) {
        setStart(start);
        setGoal(goal);
        setStepSize(step_size);
        // add the base node
        RRT_node<State> base_node;
        base_node.control_scheme = {0.0,0.0};
        base_node.node_state = m_start;
        base_node.parent_id = -1;
        m_nodes.push_back(base_node);
    }
    
    void setStepSize(double step_size) {
        m_step_size = step_size;
    }
    
    void setGoalTolerance(double tol) {
        m_goal_tolerance = tol;
    }
    
    void setGoalBias(double bias) {
        m_goal_bias = bias;
    }
    void setStart(State start) {
        m_start = start;
    }
    
    void setGoal(State goal) {
        m_goal = goal;
    }
    
    void build_RRT(int max_nodes) {
        for (int i=0; i != max_nodes; ++i) {
            std::cout << "Adding node " << i+1 << "\n";
            State q_rand;
            if (randDouble(0.0,1.0) < m_goal_bias) {
                std::copy(m_goal.begin(),m_goal.end(),q_rand.begin());
            } else {
                getRandomPosition(q_rand);
            }
            int nearest = this->findNearestNode(q_rand);
            this->extend(q_rand, nearest);
            if (m_agent.getHeuristicDistance(m_nodes.back().node_state,m_goal) < m_goal_tolerance) {
                std::cout << "Path found after " << i+1 << " nodes created. \n";
                this->writeTree();
                this->writeSolution();
                return;
            }
        }
        this->writeTree();
        std::cout << "Path not found after " << max_nodes << " nodes created. \n";
    }
    
private:
    int findNearestNode(State& q_rand) {
        typedef typename std::vector<RRT_node<State> >::iterator iter;
        
        double best_dist = std::numeric_limits<double>::infinity();
        int best = 0;
        for (iter i=m_nodes.begin(); i != m_nodes.end();++i) {
            double dist = m_agent.getHeuristicDistance(q_rand, i->node_state);
            if (dist < best_dist) {
                best_dist = dist;
                best = static_cast<int> (i-m_nodes.begin());
            }
        };
        return best;
    }
    
    void getRandomPosition(State& q_rand) {
        // change this to be dependent on the environment/agent
        m_agent.queryStateSpace(q_rand);
        m_rand.push_back(q_rand);
    }
    
    void extend(const State& q_rand, int nearest) {
        State q_near = m_nodes[nearest].node_state;
        State q_new;
        std::vector<double> controlScheme;
        m_agent.getController(q_near,q_rand,m_step_size,q_new,controlScheme);
        std::vector<State> path;
        std::vector<double> time;
        q_new = m_agent.simulate(q_near, m_step_size, controlScheme,path,time);
        RRT_node<State> new_node;
        new_node.control_scheme = controlScheme;
        new_node.node_state = q_new;
        new_node.parent_id = nearest;
        new_node.path = path;
        m_nodes.push_back(new_node);
    }
    
    void findPathBack(std::vector<int>& path, int final_id) {
        while (final_id != 0) {
            path.push_back(final_id);
            final_id = m_nodes[final_id].parent_id;
        }
        std::reverse(path.begin(),path.end());
        std::cout << "Path: ";
        for (size_t i=0; i!= path.size(); ++i) {
            std::cout << path[i] << ",";
        }
        std::cout << "\n";
    }
    
    void writeTree() {
        m_agent.writeTree(m_nodes);
    }
    
    void writeSolution() {
        m_agent.writePath(m_nodes,m_step_size);
    }
    
private:
    State m_start;
    State m_goal;
    double m_step_size;          // time taken per link
    double m_goal_bias = 0.0;
    double m_goal_tolerance = 0.5;
    Environment m_environment;
    std::vector< RRT_node<State> > m_nodes;
    std::vector<State> m_rand;
    Agent m_agent;
};


#endif /* RRTClass_hpp */
