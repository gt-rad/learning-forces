//
//  baseSimulator.h
//  physx_test
//
//  Created by YuWenhao on 01/19/16.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef __physx_test__baseSimulator__
#define __physx_test__baseSimulator__

#include <stdio.h>

#include "PxSphereManager.h"
#include "Cloth.h"
#include "RigPart.h"
#include <PxPhysicsAPI.h>
#include <Eigen/Dense>
#include "myUtils/ConfigManager.h"

class baseSimulator {
public:
    virtual void initialize();
    
    virtual void destroy();
    
    // reset to initial state
    virtual void reset() = 0;
    
    // simulate for certain steps
    virtual void simulate(int) = 0;
    
    // set parameter
    virtual void setParameter(Eigen::VectorXd) = 0;
    
    virtual int getParameterSize() = 0;
    
    virtual void getParameterBound(Eigen::VectorXd&, Eigen::VectorXd&) = 0;
    
    virtual void updateForces(int) = 0;    // update collision forces and solve gripper constraints
    
    virtual void identifyGrippedParticles() = 0;
    
    // simulated cloth
    Cloth cloth;
    
    // rigid bodies in scene
    std::vector<RigPart> rig_parts;
    
    // capsule data
    PxSphereManager sphereManager;
    
    // for cma learning
    int parameter_size = 0; // just cloth iteration number
    
    
    physx::PxPhysics*				gPhysicsSDK = NULL;			//Instance of PhysX SDK
    physx::PxFoundation*			gFoundation = NULL;			//Instance of singleton foundation SDK class
    physx::PxCooking*           gCooking = NULL;
    physx::PxDefaultErrorCallback	gDefaultErrorCallback;		//Instance of default implementation of the error callback
    physx::PxDefaultAllocator		gDefaultAllocatorCallback;	//Instance of default implementation of the allocator interface required by the SDK
    physx::PxScene*						gScene = NULL;				//Instance of PhysX Scene
    physx::PxReal							gTimeStep = 1.0f/300;		//Time-step value for PhysX simulation
};


#endif /* defined(__physx_test__baseSimulator__) */
