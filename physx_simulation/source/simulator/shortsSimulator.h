//
//  gownSimulator.h
//  physx_test
//
//  Created by YuWenhao on 10/6/15.
//  Adapted by Zackory Erickson
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef __physx_test__shortsSimulator__
#define __physx_test__shortsSimulator__

#include <stdio.h>

#include "baseSimulator.h"
#include "Cloth.h"
#include "RigPart.h"
#include <PxPhysicsAPI.h>
#include <Eigen/Dense>
#include <random>

class shortsSimulator : public baseSimulator {
public:
    void initialize(int maxSteps=0, bool enableWind=false);

    // reset to initial state
    void reset();

    void randomizeWind();

    // simulate for certain steps
    void simulate(int);

    // Creates and positions the arm based on a horizontal and vertical offset
    void positionArm(double horizontalOffset, double verticalOffset, double sidewaysOffset, std::vector<double> rotateFist={0, 0, 0, 0}, std::vector<double> rotateArm={0, 0, 0, 0}, std::vector<double> rotateArmCenter={0, 0, 0});

    void initSpline(std::vector<Eigen::VectorXd> splinePoints);

    void setVelocityFactor(double velocityFactor=1.0);

    void rotateSphereAround(int index, double angle, std::vector<double> axis, std::vector<double> center);

    PxSphereManager* getSphereManager() {
        return PxSphereManager::GetSingleton();
    }

    // Get forcemap
    std::vector<physx::PxVec4> getForcemap() {
        return PxSphereManager::GetSingleton()->getForceMap();
    }

    // Get full 3D forcemap
    std::vector<std::vector<double>> getFullForcemap() {
        return PxSphereManager::GetSingleton()->getFullForceMap();
    }

    // Get forcemap
    std::vector<physx::PxVec4> getArmSpheres() {
        return PxSphereManager::GetSingleton()->getSpheres();
    }

    // Get gripper location
    physx::PxVec3 getGripperPos() {
        return rig_parts[1].components[1]->getGlobalPose().p;
    }

    // set parameter
    void setParameter(Eigen::VectorXd);

    int getParameterSize();

    void getParameterBound(Eigen::VectorXd&, Eigen::VectorXd&);

    void updateForces(int);    // update collision forces and solve gripper constraints

    void identifyGrippedParticles();

    // stage control
    int sim_stage = 0;  // 0: gripping, 1: stop, 2: moving
    physx::PxVec3 move_step = physx::PxVec3(-0.05, 0, 0);
    physx::PxVec3 orig_move_step = physx::PxVec3(-0.05, 0, 0);
    int stepsPerSimulation = 0;
    int simulated_step = 0;
    float simulate_time = 0;

    // simulated cloth
    int cloth_solve_iteration = 1;
    int cloth_solve_iteration_aftergrip = 20;
    physx::PxVec3 random_wind;
    bool windEnabled = false;

    double damping_aftergrip = 0.0025;

    // record position and timing of force-torque data
    std::vector<double> recorded_positiosn;
    std::vector<double> recorded_time;

    // mass scale for gripped particles
    double grip_massscale = 100;

    // moving speed of linear actuator
    double moving_speed = 0.1;  // 0.1 mps
    double velocity = 1.0;

    // size of arm
    double arm_height_perturb = 0;
    double arm_horizontal_perturb = 0;

    /*double fist_radius = 0.0428046018892004;
    double wrist_radius = 0.0356159036414757;
    double elbow_radius = 0.0495531524348655;
    double shoulder_radius = 0.075;*/

    double fist_radius = 0.05;
    double wrist_radius = 0.0365;
    double elbow_radius = 0.05;
    double shoulder_radius = 0.075;

    double forearm_length = 0.31;
    double upperearm_length = 0.32;
    double arm_height = -0.13 + wrist_radius + 0.205 + arm_height_perturb;

    // range of different quantities
    //double range_arm_height_perturb[2] = {-0.205/5*3-0.06, -0.205/5*3}; // caught height
    //double range_arm_height_perturb[2] = {-0.205-0.06, 0.0};    // all height
    //double range_arm_height_perturb[2] = {-0.045, -0.015};      // good height
    double range_arm_height_perturb[2] = {-0.205-0.03, -0.205-0.0};      // miss height

    //double range_arm_horizontal_perturb[2] = {0.0, 0.03}; // miss
    //double range_arm_horizontal_perturb[2] = {0.0, 0.03}; // caught
    double range_arm_horizontal_perturb[2] = {0.0, 0.03}; // good

    double range_fist_radius[2] = {0.042, 0.044};
    double range_wrist_radius[2] = {0.0345, 0.0365};
    double range_elbow_radius[2] = {0.049, 0.051};
    double range_shoulder_radius[2] = {0.065, 0.085};
    double range_upperearm_length[2] = {0.32, 0.33};
    double range_forearm_length[2] = {0.28, 0.34};
    double range_friction[2] = {0.605, 0.655};

    bool testbit = false;
};


#endif /* defined(__physx_test__shortsSimulator__) */
