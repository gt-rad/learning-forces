//
//  RigPart.h
//  physx_test
//
//  Created by YuWenhao on 10/9/15.
//  Adapted by Zackory Erickson
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef __physx_test__RigPart__
#define __physx_test__RigPart__

#include "renderer/RenderDescriptor.h"

#include <stdio.h>
#include <PxPhysicsAPI.h>
#include <vector>
#include "myUtils/spline.hpp"
#include "Eigen/Dense"

// parts on the rig, including gripper and rod
// has force torque sensor attached
// particle constraints also here
class RigPart {
public:
    ~RigPart();
    RigPart();
    
    void updateContactForce(physx::PxVec3 orig, physx::PxVec3 force);
    
    void clearForceTorque();
    
    void addToScene(physx::PxScene*);
    
    void translate(int, physx::PxVec3, bool mod_ft = false);
    
    void rotate(int, physx::PxReal, physx::PxVec3, bool mod_ft = false);
    
    void rotateAround(int, physx::PxReal, physx::PxVec3, physx::PxVec3, bool mod_ft = false);
    
    void translateTo(int, physx::PxVec3, bool mod_ft = false);
    
    void rotateTo(int, physx::PxReal, physx::PxVec3, bool mod_ft = false);
    
    void recordInitTransform();
    
    void reset();
    
    void recordForceTorque();
    
    std::vector<RenderDescriptor* > descriptors;
    
    std::vector<std::pair<int, physx::PxVec3> > gripper_constraints;
    std::vector<physx::PxVec3> gripper_previous;
    
    physx::PxVec3 ft_center;
    physx::PxVec3 init_ft_center;
    
    physx::PxVec3 total_force;
    physx::PxVec3 total_torque;
    
    std::vector<physx::PxTransform> global_init_transforms, local_init_transforms;
    
    std::vector<physx::PxVec3> recorded_forces;
    std::vector<physx::PxVec3> recorded_torques;
    
    std::vector<physx::PxRigidStatic* > components;

    //motion spline additions
    bool is_motion_rig;
	double spline_time; //current spline traversal time
    Spline<Eigen::Vector3d> translation_spline;
	Spline<physx::PxQuat> rotation_spline;
    void setMotion(Spline<Eigen::VectorXd> _motion_spline);
    void loadMotion(std::string filename);
    void followMotion(double simTime);
	void followMotionVelocity(double speed);
};

#endif /* defined(__physx_test__RigPart__) */
