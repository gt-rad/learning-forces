//
//  Cloth.h
//  physx_test
//
//  Created by YuWenhao on 10/1/15.
//  Adapted by Zackory Erickson
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef __physx_test__Cloth__
#define __physx_test__Cloth__

#include <stdio.h>
#include "Mesh.h"
#include <PxPhysicsAPI.h>

class Cloth {
public:
    ~Cloth();
    
    // cloth properties
    std::vector<float> vstretch_stiff = {1, 1, 1, 1};
    std::vector<float> hstretch_stiff = {1, 1, 1, 1};
    std::vector<float> shear_stiff = {1, 1, 1, 1};
    std::vector<float> bend_stiff = {1, 1, 1, 1};
    float friction = 0.533909;
    float damping[3] = {0.03, 0.03, 0.03};
    // float density = 0.145;   // gown
    float density = 0.1172673881;        // sleeve
    
    float self_collision_distance = 0.01;
    float self_collision_stiff = 1.0;
    float self_friction = 0.0;
    
    unsigned int stiffpower = 0.0;
    
    float max_stretch = -1;  // used to decide if cloth deformation is invalid
    
    float color[3];
    
    Mesh cloth_mesh;
    
    Mesh initial_mesh; // for restoring initial state
    
    void createCloth(physx::PxPhysics*, physx::PxTransform transform = physx::PxTransform(), physx::PxMeshScale scale = physx::PxMeshScale());
    
    void loadMesh(std::string filename, Eigen::Vector3d translation, Eigen::Matrix3d rotation, Eigen::Vector3d scale);
    
    void updateMesh();
    
    void resetCloth();
    
    void setParameters(); // set cloth parameters use the values in the member variables
    
    physx::PxCloth* mCloth;
    
    int parameter_size = 5;
    
    void setParameter(Eigen::VectorXd);
    
    void getParameterBound(Eigen::VectorXd&, Eigen::VectorXd&);
};

#endif /* defined(__physx_test__Cloth__) */
