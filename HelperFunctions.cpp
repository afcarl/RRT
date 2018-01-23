//
//  HelperFunctions.cpp
//  Practice
//
//  Created by David Klee on 11/30/17.
//  Copyright © 2017 David Klee. All rights reserved.
//

#include "HelperFunctions.hpp"

double wrapMax(double x, double max) {
    return fmod(max + fmod(x, max), max);
}

double wrapMinMax(double x, double min, double max) {
    return min + wrapMax(x - min, max - min);
}


double randDouble(double low, double high)
{
    double temp;
    /* swap low & high around if the user makes no sense */
    if (low > high)
    {
        temp = low;
        low = high;
        high = temp;
    }
    
    /* calculate the random number & return it */
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<float> dis(0.0,1.0);
    
    temp = dis(gen)*(high-low) + low;
    return temp;
}
