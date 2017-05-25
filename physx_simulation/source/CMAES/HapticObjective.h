//
//  HapticObjective.h
//  physx_test
//
//  Created by YuWenhao on 1/9/16.
//  Copyright (c) 2016 YuWenhao. All rights reserved.
//

#ifndef __physx_test__HapticObjective__
#define __physx_test__HapticObjective__

#include <stdio.h>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "LearnerObjective.h"
#include "simulator/gownSimulator.h"

class HapticObjective : public LearnerObjective {
public:
    virtual double evalObjective(baseSimulator*);
    
    void ReadData(std::string filename);
    
    void Clear();
    
    std::vector<double> exp_time;
    std::vector<double> exp_position;
    std::vector<double> exp_fmove;
    std::vector<double> exp_flateral;
    std::vector<double> exp_fgravity;
    
    void getInterpolationFrac(const std::vector<double>&, double val, int& ind1, double& frac1, int& ind2, double& frac2);
    
    double sanityCheck();
private:
    double evalObj(const std::vector<double>& sim_position, const std::vector<double>& sim_fmove, const std::vector<double>& sim_fgravity);
};


#endif /* defined(__physx_test__HapticObjective__) */
