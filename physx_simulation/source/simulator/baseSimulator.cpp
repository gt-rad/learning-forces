//
//  baseSimulator.cpp
//  physx_test
//
//  Created by YuWenhao on 1/19/16.
//  Copyright (c) 2016 YuWenhao. All rights reserved.
//

#include <stdio.h>
#include <iostream>
#include "baseSimulator.h"


using namespace physx;
using namespace std;

void baseSimulator::initialize() {
    //Creating foundation for PhysX
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);
    
    //Creating instance of PhysX SDK
    gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale() );
    
    if(gPhysicsSDK == NULL)
    {
        cerr << "Error creating PhysX3 device, Exiting..." << endl;
        exit(1);
    }
    
    gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));
    if (!gCooking) {
        cerr << "PxCreateCooking failed!" << endl;
        exit(1);
    }
    
    //Creating scene
    PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());		//Descriptor class for scenes
    
    
    sceneDesc.gravity		= PxVec3(0.0f, -9.8f, 0.0f);			//Setting gravity
    sceneDesc.cpuDispatcher = PxDefaultCpuDispatcherCreate(1);		//Creating default CPU dispatcher for the scene
    sceneDesc.filterShader  = PxDefaultSimulationFilterShader;		//Creating default collision filter shader for the scene
    
    sceneDesc.frictionType = PxFrictionType::eTWO_DIRECTIONAL;
    
    gScene = gPhysicsSDK->createScene(sceneDesc);					//Creating a scene
}

void baseSimulator::destroy() {
    gPhysicsSDK->release();			//Removes any actors,  particle systems, and constraint shaders from this scene
    gFoundation->release();			//Destroys the instance of foundation SDK
}