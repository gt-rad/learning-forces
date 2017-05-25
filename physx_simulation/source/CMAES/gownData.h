//
//  gownData.h
//  physx_test
//
//  Created by YuWenhao on 2/4/16.
//  Copyright (c) 2016 YuWenhao. All rights reserved.
//

#ifndef __physx_test__gownData__
#define __physx_test__gownData__

#include "CMAData.h"

class gownData : public CMAData {
public:
    enum PARAMETERS {
        STRETCH,
        SHEAR,
        BEND,
        
        FRICTION,
        SELFCOLDIST,
        SELFCOLFRICTION,
        
        STIFFPOWER,
        ITERATION,
        WIND,
        
        ARM_HEIGHT_PERT_GOOD,
        ARM_HEIGHT_PERT_MISS,
        ARM_HEIGHT_PERT_CAUG,
        
        ARM_HORIZONTAL_PERT_GOOD,
        ARM_HORIZONTAL_PERT_MISS,
        ARM_HORIZONTAL_PERT_CAUG,
        
        FIST_RADIUS,
        WRIST_RADIUS,
        ELBOW_RADIUS,
        SHOULDER_RADIUS,
        FOREARM_LENGTH
    };
    
    double* parameters = NULL;
    
    gownData();
    virtual ~gownData();
    virtual void ReadFromFile(const string& filename);
    virtual int GetNumParameters() const;
    virtual void GetParameterLowerBounds(double* lb);
    virtual void GetParameterUpperBounds(double* ub);
    virtual void FromParameterSetting(double* param);
    virtual void getParameter(double* pm);
    bool useParameter(PARAMETERS pm);
    
};

#endif /* defined(__physx_test__gownData__) */
