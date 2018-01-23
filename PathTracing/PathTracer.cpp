//
//  PathTracer.cpp
//  Practice
//
//  Created by David Klee on 12/7/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#include "PathTracer.hpp"


//void PathTracer::initialize(state_type& start_state)
//{
//    m_target_path.initialize();
//    xy_type start_xy;
//    getXYfromState(start_state, start_xy);
//    m_mirror_path.push_back(start_xy);
//}
//
//void PathTracer::scanControls()
//{
//    state_type start = {0.0,0.0,0.5,0.0};
//    xy_type start_xy;
//    getXYfromState(start, start_xy);
//    int start_ID = m_target_path.getClosestPointID(start_xy);
//    double tau1,tau2,best_tau1,best_tau2;
//    best_tau1 = 0; best_tau2 = 0;
//    double best_score = 0.0;
//    std::vector<state_type> path_record;
//    std::vector<xy_type> path_record_xy;
//    for (int i=-5;i<11;++i) {
//        for (int j=-5;j<11;j++) {
//            tau1 = static_cast<double>(i);
//            tau2 = static_cast<double>(j);
//            simulate(start, 0.1, tau1, tau2, path_record);
//            convertPathToXY(path_record, path_record_xy);
//            double score = m_target_path.getMovementScore(path_record_xy,start_ID);
//            if (score > best_score) {
//                best_tau1 = tau1;
//                best_tau2 = tau2;
//                best_score = score;
//            }
//            path_record.clear();
//            path_record_xy.clear();
//        }
//    }
//    simulate(start, 0.1, best_tau1, best_tau2, path_record);
//    convertPathToXY(path_record, path_record_xy);
//    
//    
//}

