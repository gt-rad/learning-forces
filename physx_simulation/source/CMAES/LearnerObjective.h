//
//  LearnerObjective.h
//  physx_test
//
//  Created by YuWenhao on 10/6/15.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef __physx_test__LearnerObjective__
#define __physx_test__LearnerObjective__

#include <stdio.h>
#include "simulator/baseSimulator.h"

class LearnerObjective {
public:
    virtual double evalObjective(baseSimulator*) = 0;
};

#endif /* defined(__physx_test__LearnerObjective__) */
