//
//  RenderDiscriptor.cpp
//  physx_test
//
//  Created by YuWenhao on 10/14/15.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#include "RenderDescriptor.h"

using namespace physx;

CapsuleDescriptor::CapsuleDescriptor(double h_height, double rad) : RenderDescriptor(), half_height(h_height), radius(rad) {
    type = rCapsule;
    
}

BoxDescriptor::BoxDescriptor(double h_width, double h_height, double h_length) : RenderDescriptor(), half_width(h_width), half_length(h_length), half_height(h_height) {
    type = rBox;
}