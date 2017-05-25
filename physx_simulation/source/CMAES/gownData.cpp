//
//  gownData.c
//  physx_test
//
//  Created by YuWenhao on 2/4/16.
//  Copyright (c) 2016 YuWenhao. All rights reserved.
//

#include "gownData.h"
#include <vector>

#define TOTAL_PARAMETERS 20
const float MIN_VALUES[] = {0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 5.0, 0.0, 0.0, -0.04, -0.04, -0.06, -0.03, -0.03, -0.03,     0.04, 0.03, 0.04, 0.065, 0.28};
const float MAX_VALUES[] = {1.0, 1.0, 1.0, 0.8, 0.015, 1.0, 10.0, 20.0, 1.0, 0.0, 0.0, 0.0, 0.03, 0.03, 0.03,      0.06, 0.043, 0.06, 0.085, 0.35};
const float USE_PARAMETER[] = {1,1,1,1,0,1,0,0,0, 1,1,0, 0,0,0, 0,0,0,0,0};
//const float USE_PARAMETER[] = {0,0,0,0,0,0,0,0,0,1,1,0, 1,1,1, 1,1,1,1,1};
//const float USE_PARAMETER[] = {1,1,1,1,0,1,0,0,0,1,1,0, 1,1,0, 0,0,0,0,0};

using namespace std; 

gownData::gownData() {
    parameters = new double[GetNumParameters()];
}

gownData::~gownData() {
    if (!parameters) {
        delete []parameters;
    }
}

void gownData::ReadFromFile(const string& filename) {
    
}

int gownData::GetNumParameters() const {
    int rst = 0;
    for (int i = 0; i < TOTAL_PARAMETERS; i++) {
        if (USE_PARAMETER[i]) {
            rst += 1;
        }
    }
    return rst;
}

void gownData::GetParameterLowerBounds(double* lb) {
    std::vector<double> lowbound;
    for (int i = 0; i < TOTAL_PARAMETERS; i++) {
        if (USE_PARAMETER[i]) {
            lowbound.push_back(MIN_VALUES[i]);
        }
    }
    for (int i = 0; i < lowbound.size(); i++) {
        lb[i] = lowbound[i];
    }
}

void gownData::GetParameterUpperBounds(double* ub) {
    std::vector<double> upbound;
    for (int i = 0; i < TOTAL_PARAMETERS; i++) {
        if (USE_PARAMETER[i]) {
            upbound.push_back(MAX_VALUES[i]);
        }
    }
    for (int i = 0; i < upbound.size(); i++) {
        ub[i] = upbound[i];
    }
}

void gownData::FromParameterSetting(double* param) {
    for (int i = 0; i < GetNumParameters(); i++) {
        parameters[i] = param[i];
    }
}

void gownData::getParameter(double* pm) {
    for (int i = 0; i < GetNumParameters(); i++) {
        pm[i] = parameters[i];
    }
}

bool gownData::useParameter(PARAMETERS pm) {
    return USE_PARAMETER[pm];
}