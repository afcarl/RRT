//
//  HelperFunctions.hpp
//  Practice
//
//  Created by David Klee on 11/30/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#ifndef HelperFunctions_hpp
#define HelperFunctions_hpp

#include <stdio.h>
#include <cmath>
#include <boost/array.hpp>
#include <array>
#include <iostream>
#include <vector>
#include <random>

typedef std::array<double,2> xy_type;
typedef std::array<double,2> interval_type;           // t_start, t_end for time based obstacles
typedef std::array<xy_type,3> triangle_type;
typedef std::array<double,3> xyt_type;
typedef std::array<double,3> xydir_type;

template<class Vec>
double getAngleFromVector(Vec& vec) {
    // returns angle on the range [0,2pi)
    double angle;
    if (vec[0] < 0.0001 and vec[0] > -0.0001) { // angle is either pi/2 or -pi/2
        if (vec[1] >= 0) {
            angle = M_PI_2;
        } else {
            angle = -M_PI_2;
        }
    } else {
        if (vec[0] >= 0 and vec[1] >= 0) { // angle is within atan range of -pi/2 to +pi/2
            angle = atan(vec[1]/abs(vec[0]));
        } else if (vec[0] >= 0 and vec[1] < 0) {
            angle = 2*M_PI + atan(vec[1]/abs(vec[0]));
        } else { // actual angle is within range +pi/2 to -pi/2
            angle = M_PI - atan(vec[1]/abs(vec[0]));
        }
    }
    return angle;
}

template<class Vec>
double getVectorProjection(Vec& vec,Vec& vec_projected_on) {
    return getDotProduct(vec, vec_projected_on)/getMagnitude(vec_projected_on);
}

template<class Vec>
void rotateVector(Vec& vec, double rotation,bool rotateCCW) {
    if (rotateCCW) {
        vec[0] = vec[0]*cos(rotation)-vec[1]*sin(rotation);
        vec[1] = vec[0]*sin(rotation)+vec[1]*cos(rotation);
    } else {
        vec[0] = vec[0]*cos(rotation)+vec[1]*sin(rotation);
        vec[1] = -vec[0]*sin(rotation)+vec[1]*cos(rotation);
    }
}

template<class Vec>
double getEuclideanDistance(Vec& point_a,Vec& point_b) {
    double dist = sqrt(pow((point_a[0]-point_b[0]),2.0) + pow((point_a[1]-point_b[1]),2));
    return dist;
}

template<class Vec>
double getMagnitude(Vec& vec) {
    int len = static_cast<int> (vec.size());
    double result = 0.0;
    for (int i = 0; i  < len; i++) {
        result += vec[i]*vec[i];
    }
    return sqrt(result);
}

template<class Vec>
double getDotProduct(Vec& vec_1,Vec& vec_2) {
    int len = static_cast<int> (vec_1.size());
    double result = 0.0;
    for (int i = 0; i  < len; i++) {
        result += vec_1[i]*vec_2[i];
    }
    return result;
}

double wrapMax(double x, double max);
double wrapMinMax(double x, double min, double max);

template<class Vec>
double getSlope(Vec& pointA, Vec& pointB) {
    if (abs(pointB[0]-pointA[0]) > 0.0001) {
        return (pointB[1]-pointA[1])/(pointB[0]-pointA[0]);
    } else {
        return std::numeric_limits<double>::infinity();
    };
}

double randDouble(double low, double high);


#endif /* HelperFunctions_hpp */

