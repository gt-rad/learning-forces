//
//  Renderer.h
//  physx_test
//
//  Created by YuWenhao on 10/1/15.
//  Adapted by Zackory Erickson
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef __physx_test__Renderer__
#define __physx_test__Renderer__

#include <stdio.h>
#include <vector>
#include "../utils/opengl.h"
#include "../simulator/Mesh.h"
#include "../simulator/RigPart.h"

class Renderer {
public:
    void renderMesh(Mesh&, std::vector<float> color, bool wire = true, bool seqinv = false);
    
    void renderRigPart(RigPart*, std::vector<float> color);

    void renderRigMotion(RigPart*, std::vector<float> color);
    
    void renderShape(RenderDescriptor*, std::vector<float> color);

    void renderForceMap(float alpha=0.25f);
    
    void renderSphereManager(std::vector<float> color, bool drawSpheres=true);
};


#endif /* defined(__physx_test__Renderer__) */
