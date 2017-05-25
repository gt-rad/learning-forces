//
//  RenderDiscriptor.h
//  physx_test
//
//  Created by YuWenhao on 10/14/15.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef __physx_test__RenderDiscriptor__
#define __physx_test__RenderDiscriptor__

#include <stdio.h>
#include <PxPhysicsAPI.h>

class RenderDescriptor {
public:
    enum RenderType {rCapsule, rBox};
    
    RenderType type;
    
    physx::PxVec3 translation = physx::PxVec3(0);
    physx::PxQuat rotation = physx::PxQuat(0, 0, 0, 1);
    physx::PxVec3 scale = physx::PxVec3(1);
};

class CapsuleDescriptor : public RenderDescriptor {
public:
    CapsuleDescriptor(double, double);
    double half_height;
    double radius;
};

class BoxDescriptor : public RenderDescriptor {
public:
    BoxDescriptor(double, double, double);
    double half_width;
    double half_length;
    double half_height;
};

#endif /* defined(__physx_test__RenderDiscriptor__) */
