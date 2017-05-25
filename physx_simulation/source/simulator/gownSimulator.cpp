//
//  gownSimulator.cpp
//  physx_test
//
//  Created by YuWenhao on 10/6/15.
//  Adapted by Zackory Erickson
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#include "gownSimulator.h"
#include <iostream>

#include "myUtils/mathlib.h"

#include <unistd.h>
#include "config.h"
#include "myUtils/ConfigManager.h"
#include "myUtils/spline.hpp"

using namespace std;
using namespace Eigen;
using namespace physx;

void gownSimulator::initialize(int maxSteps, bool enableWind) {
    baseSimulator::initialize();

    stepsPerSimulation = maxSteps;
    windEnabled = enableWind;

    // Initialize random seed
    srand(getpid());

    // Randomize wind
    randomizeWind();

    orig_move_step[0] = - moving_speed * gTimeStep;
    move_step[0] = - moving_speed * gTimeStep;

    // create capsule
    rig_parts.resize(2);

    // create gripper
    PxMaterial* material = gPhysicsSDK->createMaterial(1.0f,1.0f,0.5f);
    PxRigidStatic* new_rigid = NULL;
    new_rigid = NULL;
    while (new_rigid == NULL) {
        new_rigid = gPhysicsSDK->createRigidStatic(PxTransform(PxVec3(0)));
    }
    rig_parts[1].components.push_back(new_rigid);
    new_rigid = NULL;
    while (new_rigid == NULL) {
        new_rigid = gPhysicsSDK->createRigidStatic(PxTransform(PxVec3(0)));
    }
    rig_parts[1].components.push_back(new_rigid);
    rig_parts[1].components[0]->createShape(PxBoxGeometry(0.04, 0.005, 0.06), *material);
    rig_parts[1].components[1]->createShape(PxBoxGeometry(0.04, 0.005, 0.06), *material);
    rig_parts[1].descriptors.push_back(new BoxDescriptor(0.04, 0.005, 0.06));
    rig_parts[1].descriptors.push_back(new BoxDescriptor(0.04, 0.005, 0.06));
    rig_parts[1].ft_center = PxVec3(0, 0.005, 0);
    rig_parts[1].translate(0, PxVec3(-forearm_length+0.48, 0.2, 0), true);
    rig_parts[1].translate(1, PxVec3(-forearm_length+0.48, 0.06, 0));
	//rig_parts[1].rotateTo(1, sqrt(0.5), PxVec3(0,0,sqrt(0.5)), false);

    rig_parts[1].addToScene(gScene);
    rig_parts[1].recordInitTransform();

    //testing spline with rotation
	/*
	Spline<Eigen::VectorXd> gripper_motion;
	Eigen::VectorXd p0(7); p0 << -forearm_length+0.48	, 0.2,	0,		0,1,0,0;
	Eigen::VectorXd p1(7); p1 << -forearm_length		, 0.2,	0.5,	1.57,0,1,0;
	Eigen::VectorXd p2(7); p2 << -forearm_length-0.5	, 0.2,	-0.5,	1.57,1,0,0;
	gripper_motion.insert(0, p0);
	gripper_motion.insert(1, p1);
	gripper_motion.insert(2, p2);
	rig_parts[1].setMotion(gripper_motion);
    //done testing spline

	//testing spline without rotation
	Spline<Eigen::VectorXd> gripper_motion;
	//							 tx						  ty	tz		w x y z
	Eigen::VectorXd p0(7); p0 << -forearm_length+0.48	, 0.2,	0,		0,1,0,0; //create a new control point. [0,1,2] are translation, [3,4,5,6] are angle, axis rotation (see above example)
	Eigen::VectorXd p1(7); p1 << -forearm_length		, 0.2,	0.1,	0,1,0,0;
	Eigen::VectorXd p2(7); p2 << -forearm_length-0.3	, 0.2,	-0.1,	0,1,0,0;
	gripper_motion.insert(0, p0); //insert a new spline control point at a specificy time (local time in phase 2)
	gripper_motion.insert(0.5, p1);
	gripper_motion.insert(1, p2);
	rig_parts[1].setMotion(gripper_motion);*/
	//done testing spline

    // create arm
    positionArm(0, 0, 0);
    PxSphereManager* sphereManager = PxSphereManager::GetSingleton();
    sphereManager->addCapsule(1, 2);
    sphereManager->addCapsule(2, 3);

    // Remove gravity and clear recorded variables
    gScene->setGravity(PxVec3(0));

    recorded_positiosn.clear();
    recorded_time.clear();
    simulate_time = 0;

    // create cloth
    PxTransform gPose = PxTransform(PxVec3(0,0,0));
    Matrix3d rot;
    rot = AngleAxisd(1.57, Vector3d::UnitY()) * AngleAxisd(-1.2, Vector3d::UnitX()) * AngleAxisd(-3.14, Vector3d::UnitZ());
    cloth.density = 0.145;

    string gown_path;
    DecoConfig::GetSingleton()->GetString("GOWN", "MeshFilePath", gown_path);
    cloth.loadMesh(string(PHYSX_ROOT_PATH)+gown_path, Vector3d(-forearm_length + 0.71, -0.0, -0.3), rot, Vector3d(1.679, 1.679, 1.679));
    cloth.createCloth(gPhysicsSDK, gPose);

    DecoConfig::GetSingleton()->GetFloat("GOWN", "VStretch", cloth.vstretch_stiff[0]);
    DecoConfig::GetSingleton()->GetFloat("GOWN", "HStretch", cloth.hstretch_stiff[0]);
    DecoConfig::GetSingleton()->GetFloat("GOWN", "Shear", cloth.shear_stiff[0]);
    DecoConfig::GetSingleton()->GetFloat("GOWN", "Bend", cloth.bend_stiff[0]);
    DecoConfig::GetSingleton()->GetFloat("GOWN", "Friction", cloth.friction);
    DecoConfig::GetSingleton()->GetFloat("GOWN", "SelfFriction", cloth.self_friction);
    DecoConfig::GetSingleton()->GetFloat("GOWN", "SelfCollisionDist", cloth.self_collision_distance);
    DecoConfig::GetSingleton()->GetInt("GOWN", "Iteration", cloth_solve_iteration_aftergrip);
    DecoConfig::GetSingleton()->GetDouble("GOWN", "Damping", damping_aftergrip);
    DecoConfig::GetSingleton()->GetDouble("GOWN", "HeightOffset", arm_height_perturb);
    DecoConfig::GetSingleton()->GetDouble("GOWN", "FistRadius", fist_radius);
    DecoConfig::GetSingleton()->GetDouble("GOWN", "WristRadius", wrist_radius);
    DecoConfig::GetSingleton()->GetDouble("GOWN", "ElbowRadius", elbow_radius);
    DecoConfig::GetSingleton()->GetDouble("GOWN", "ShoulderRadius", shoulder_radius);
    DecoConfig::GetSingleton()->GetDouble("GOWN", "ForearmLength", forearm_length);
    DecoConfig::GetSingleton()->GetDouble("GOWN", "UpperArmLength", upperearm_length);

    vector<double> range;
    DecoConfig::GetSingleton()->GetDoubleVector("Sample", "RangeHeightOffset", range);
    range_arm_horizontal_perturb[0] = range[0];
    range_arm_horizontal_perturb[1] = range[1];

    arm_height = -0.13 + wrist_radius + 0.205 + arm_height_perturb;

    cloth.damping[0] = 0.03;
    cloth.damping[1] = 0.03;
    cloth.damping[2] = 0.03;
    cloth.stiffpower = (unsigned int)7;
    cloth.setParameters();

    gScene->addActor(*cloth.mCloth);

    sphereManager->recordInitSpheres();
    sphereManager->setUpCapsules(cloth.mCloth);

    cout << "using stretch: " << cloth.vstretch_stiff[0] << endl;
    cout << "using friction: " << cloth.friction << endl;
    cout << "using iteration number: " << cloth_solve_iteration_aftergrip << endl;
    cout << "using stiff power: " << cloth.stiffpower << endl;
    cout << "initialize ok" << endl;
    cout << "using damping: " << cloth.damping[0] << endl;
    cout << "using self_collision_distance: " << cloth.self_collision_distance << endl;
}

// Creates and positions the arm based on a horizontal and vertical offset
void gownSimulator::positionArm(double horizontalOffset, double verticalOffset, double sidewaysOffset, vector<double> rotateFist, vector<double> rotateArm, vector<double> rotateArmCenter) {
    PxSphereManager* sphereManager = PxSphereManager::GetSingleton();
    sphereManager->clearSpheres();
    arm_height = -0.13 + wrist_radius + 0.205 + verticalOffset;
    // Add each sphere to scene to create the full arm
    sphereManager->addSphere(PxVec3(-fist_radius+horizontalOffset, arm_height, sidewaysOffset), fist_radius); // fist
    sphereManager->addSphere(PxVec3(-fist_radius+horizontalOffset, arm_height, sidewaysOffset), wrist_radius); // wrist
    sphereManager->addSphere(PxVec3(-forearm_length-elbow_radius+horizontalOffset, arm_height, sidewaysOffset), elbow_radius); // elbow
    sphereManager->addSphere(PxVec3(-forearm_length-elbow_radius+horizontalOffset, arm_height, sidewaysOffset - upperearm_length), shoulder_radius); // shoulder

    // Rotate fist and forearm around the elbow center location
    if (rotateFist[3] != 0 && (rotateFist[0] != 0 || rotateFist[1] != 0 || rotateFist[2] != 0)) {
        double angle = rotateFist[3];
        PxVec3 axis = PxVec3(rotateFist[0], rotateFist[1], rotateFist[2]);
        PxVec3 elbowCenter = sphereManager->getSpheres()[2].getXYZ();
        sphereManager->rotateAround(0, angle, axis, elbowCenter);
        sphereManager->rotateAround(1, angle, axis, elbowCenter);
    }

    // Rotate entire arm around a given center location
    if (rotateArm[3] != 0 && (rotateArm[0] != 0 || rotateArm[1] != 0 || rotateArm[2] != 0)) {
        double angle = rotateArm[3];
        PxVec3 axis = PxVec3(rotateArm[0], rotateArm[1], rotateArm[2]);
        PxVec3 center = PxVec3(rotateArmCenter[0], rotateArmCenter[1], rotateArmCenter[2]);
        for (int j = 0; j < sphereManager->getSpheres().size(); j++) {
            sphereManager->rotateAround(j, angle, axis, center);
        }
    }
}

void gownSimulator::initSpline(vector<Eigen::VectorXd> splinePoints) {
	Spline<Eigen::VectorXd> gripper_motion;
    PxVec3 pos = rig_parts[1].components[0]->getGlobalPose().p;
    Eigen::VectorXd start(7); start << pos.x, pos.y, pos.z, 0, 1, 0, 0;
    gripper_motion.insert(0, start);
    for (int i = 0; i < splinePoints.size(); i++) {
        splinePoints[i][0] += -forearm_length + 0.48;
        splinePoints[i][1] += 0.2;
        gripper_motion.insert((i + 1) / (float) splinePoints.size(), splinePoints[i]);
    }
	rig_parts[1].setMotion(gripper_motion);
}

void gownSimulator::setVelocityFactor(double velocityFactor) {
    orig_move_step[0] = - moving_speed * gTimeStep;
    move_step[0] = - moving_speed * gTimeStep * velocityFactor;
    velocity = velocityFactor;
}

void gownSimulator::rotateSphereAround(int index, double angle, vector<double> axis, vector<double> center) {
    PxSphereManager* sphereManager = PxSphereManager::GetSingleton();
    sphereManager->rotateAround(index, angle, PxVec3(axis[0], axis[1], axis[2]), PxVec3(center[0], center[1], center[2]));
    // Update PhysX conical frustums to match new sphere location
    sphereManager->updateCapsules(cloth.mCloth);
}

// reset to initial state
void gownSimulator::reset() {
    cout << "resetting\n";
    // reset cloth mesh
    cloth.resetCloth();
    // reset physx cloth
    PxClothParticleData* particle_data = cloth.mCloth->lockParticleData(PxDataAccessFlag::eWRITABLE);
    for (int i = 0; i < cloth.cloth_mesh.particles.size(); i++) {
        PxVec3 init_pos = PxVec3(cloth.cloth_mesh.particles[i].pos(0), cloth.cloth_mesh.particles[i].pos(1), cloth.cloth_mesh.particles[i].pos(2));
        particle_data->particles[i].pos = init_pos;
        particle_data->particles[i].invWeight = 1.0/cloth.cloth_mesh.particles[i].mass;
        particle_data->previousParticles[i].pos = init_pos;
        particle_data->previousParticles[i].invWeight = 1.0/cloth.cloth_mesh.particles[i].mass;
    }
    particle_data->unlock();

    if (sim_stage != 0) {   // add back the gripper collision
        gScene->addActor(*rig_parts[1].components[0]);
        gScene->addActor(*rig_parts[1].components[1]);
    }

    sim_stage = 0;
    gScene->setGravity(PxVec3(0));
    
    // Reset gripper
    rig_parts[1].gripper_constraints.clear();
    rig_parts[1].gripper_previous.clear();
    for (int i = 0; i < rig_parts.size(); i++) {
        rig_parts[i].reset();
    }

    PxSphereManager::GetSingleton()->resetPos();
    PxSphereManager::GetSingleton()->updateCapsules(cloth.mCloth);

    cloth_solve_iteration = 1;

    // Reset cloth damping. TODO: This doesn't change and thus could be removed.
    cloth.damping[0] = 0.03;
    cloth.damping[1] = 0.03;
    cloth.damping[2] = 0.03;
    cloth.mCloth->setDampingCoefficient(PxVec3(cloth.damping[0], cloth.damping[1], cloth.damping[2]));

    // Randomize wind
    randomizeWind();

    // Clear recorded data
    recorded_positiosn.clear();
    recorded_time.clear();
    simulate_time = 0;
    simulated_step = 0;

    int isSampling;
    DecoConfig::GetSingleton()->GetBool("Sample", "IsSampling", isSampling);
    if (isSampling) {
        // reset the arm with randomized size
        /*arm_height_perturb = RandDouble(range_arm_height_perturb[0], range_arm_height_perturb[1]);
        arm_horizontal_perturb = RandDouble(range_arm_horizontal_perturb[0], range_arm_horizontal_perturb[1]);
         fist_radius = RandDouble(range_fist_radius[0], range_fist_radius[1]);
         wrist_radius = RandDouble(range_wrist_radius[0], range_wrist_radius[1]);
         elbow_radius = RandDouble(range_elbow_radius[0], range_elbow_radius[1]);
         shoulder_radius = RandDouble(range_shoulder_radius[0], range_shoulder_radius[1]);
         upperearm_length = RandDouble(range_upperearm_length[0], range_upperearm_length[1]);
         forearm_length = RandDouble(range_forearm_length[0], range_forearm_length[1]);
         cloth.friction = RandDouble(range_friction[0], range_friction[1]);
         cloth.setParameters();*/
        positionArm(0, 0, 0);
    }
}

int gownSimulator::getParameterSize() {
    return cloth.parameter_size + parameter_size;
}

void gownSimulator::getParameterBound(VectorXd& min, VectorXd& max) {
    VectorXd cloth_min, cloth_max;
    cloth.getParameterBound(cloth_min, cloth_max);
    min.resize(getParameterSize());
    max.resize(getParameterSize());
    min.segment(0, cloth.parameter_size) = cloth_min;
    max.segment(0, cloth.parameter_size) = cloth_max;
}

void gownSimulator::randomizeWind() {
    if (windEnabled) {
        // Initialize random gust of wind that occurs directly before dressing
        double randx = (rand()*1.0/RAND_MAX-0.5)/6;
        double randy = (rand()*1.0/RAND_MAX-0.5)/6;
        double randz = (rand()*1.0/RAND_MAX-0.5)/6;
        random_wind = PxVec3(randx, randy, randz);
        // cout << "Wind: " << randx << ", " << randy << ", " << randz << endl;
    } else
        random_wind = PxVec3(0);
}

// simulate for certain steps
void gownSimulator::simulate(int steps) {
    if (sim_stage == 0) {   // gripper closing
        PxVec3 dif = -rig_parts[1].components[1]->getGlobalPose().p + rig_parts[1].components[0]->getGlobalPose().p;
        double thickness = ((BoxDescriptor*)rig_parts[1].descriptors[0])->half_height * 2;

        if (dif.magnitude() > thickness) {
            rig_parts[1].translate(1, dif / 60);
        } else {
            sim_stage = 1;
            gScene->setGravity(PxVec3(0, -9.8, 0));
            identifyGrippedParticles();
        }
    } else if (sim_stage == 1 && simulated_step > 500) {
        sim_stage = 2;
        cloth.damping[0] = damping_aftergrip;
        cloth.damping[1] = damping_aftergrip;
        cloth.damping[2] = damping_aftergrip;
        cloth.mCloth->setDampingCoefficient(PxVec3(cloth.damping[0], cloth.damping[1], cloth.damping[2]));
    } else if (sim_stage == 1 && windEnabled && simulated_step > 180 && simulated_step < 250) {
        PxClothParticleData* data = cloth.mCloth->lockParticleData(PxDataAccessFlag::eWRITABLE);
        float dt = cloth.mCloth->getPreviousTimeStep();
        for(PxU32 i = 0, n = cloth.mCloth->getNbParticles(); i < n; ++i)
        {
            int isSampling;
            DecoConfig::GetSingleton()->GetBool("Sample", "IsSampling", isSampling);
            if (isSampling)
                data->previousParticles[i].pos -= random_wind * dt;
        }
        data->unlock();
    } else if (sim_stage == 1 && simulated_step >= 300 && simulated_step < 400) {
        // move arm to the right place
        PxSphereManager* sphereManager = PxSphereManager::GetSingleton();
        /*for (int s = 0; s < sphereManager->getNumSphere(); s++) {
            sphereManager->translate(s, PxVec3(0.1/(400-300), 0, 0));
        }*/
        sphereManager->updateCapsules(cloth.mCloth);
    }
    for (int i = 0; i < steps; i++) {
	
        if (sim_stage == 2 && rig_parts[1].is_motion_rig){
            //update the rig based on the current simulation time and the motion profile
            // rig_parts[1].followMotion((simulated_step - 500) / (float) (stepsPerSimulation - 500));
            rig_parts[1].followMotionVelocity(0.1 * gTimeStep * velocity);
            // cout << (simulated_step - 500) / (float) (stepsPerSimulation - 500) << ", " << simulated_step << endl;
        } else if (sim_stage == 2 && rig_parts[1].components[1]->getGlobalPose().p.x-0.04 >= 0) {
            rig_parts[1].translate(0, orig_move_step);
            rig_parts[1].translate(1, orig_move_step, true);
        } else if (sim_stage == 2 /*&& simulated_step < 1200*/) { // gripper moving
            rig_parts[1].translate(0, move_step);
            rig_parts[1].translate(1, move_step, true);
        }
        cloth.mCloth->setSolverFlags(64+32+16+8+4+2+1);
        gScene->simulate(gTimeStep);	//Advances the simulation by 'gTimeStep' time
        gScene->fetchResults(true);
        for (int j = 0; j < cloth_solve_iteration - 1; j++) {
            cloth.mCloth->setSolverFlags(32+16+8);
            gScene->simulate(gTimeStep);	//Advances the simulation by 'gTimeStep' time
            gScene->fetchResults(true);
        }
        // put collision step as the last
        /*cloth.mCloth->setSolverFlags(32);
        gScene->simulate(gTimeStep);	//Advances the simulation by 'gTimeStep' time
        gScene->fetchResults(true);*/

        simulate_time += gTimeStep;
        updateForces(steps);
    }
    cloth.updateMesh();

    if (cloth.max_stretch > 5) {
        //cout << "Invalid deformation has occurred\n";
        //simulated_step = -100;
        //return;
    }

    if (sim_stage == 2 && (rig_parts[1].components[1]->getGlobalPose().p.x-0.04 < 0 || rig_parts[1].is_motion_rig)) {
        double total_mass = 0;
        for (int i = 0; i < cloth.cloth_mesh.particles.size(); i++) {
            total_mass += cloth.cloth_mesh.particles[i].mass;
        }

        for (int j = 0; j < rig_parts.size(); j++) {
            for (int i = 0; i < rig_parts[j].gripper_constraints.size(); i++) {
                total_mass -= cloth.cloth_mesh.particles[rig_parts[j].gripper_constraints[i].first].mass * (grip_massscale-1) / grip_massscale;
            }
        }

        // mimic equilibrium control
        //rig_parts[0].rotateAround(0, -0.52333333333/700, PxVec3(0, 0, 1), PxVec3(0.1325-2*forearm_length-capsule_radius, capsule_height, 0));
        //rig_parts[0].rotateAround(rig_parts[0].components.size()-1, -0.52333333333/700, PxVec3(0, 0, 1), PxVec3(0.1325-2*forearm_length-capsule_radius, capsule_height, 0));

        recorded_positiosn.push_back(rig_parts[1].components[0]->getGlobalPose().p.x);
        recorded_time.push_back(simulate_time);

        // zero out the mass of cloth
        rig_parts[1].total_force[1] += total_mass * 9.8;

        for (int i = 0; i < rig_parts.size(); i++) {
            rig_parts[i].recordForceTorque();
        }
        if (stepsPerSimulation == 0 && (abs(rig_parts[1].total_force[0]) > 10 || abs(rig_parts[1].total_force[1]) > 10 || rig_parts[1].components[1]->getGlobalPose().p.x-0.04 < -forearm_length+0.44-0.85)) {
            simulated_step = -100;
        } else if (stepsPerSimulation == -1 && rig_parts[1].components[1]->getGlobalPose().p.x-0.04 < -forearm_length+0.44-0.85) {
            simulated_step = -100;
        } else if (stepsPerSimulation > 0 && simulated_step > stepsPerSimulation) {
            simulated_step = -100;
        }
        //cout << "moved distance: " << forearm_length - rig_parts[1].components[1]->getGlobalPose().p.x << endl;
    }
    //cout << "vert force: " << rig_parts[1].total_force[1] << "  total weight: " << total_mass * 9.8 << endl;
    
    simulated_step++;
}

void gownSimulator::updateForces(int steps) {    // update collision forces and solve gripper constraints
    PxSphereManager* sphereManager = PxSphereManager::GetSingleton();
    sphereManager->clearForceMap();

    // update force
    for (int i = 0; i < rig_parts.size(); i++) {
        rig_parts[i].clearForceTorque();
    }
    float* contact_forces = cloth.mCloth->getContactForces();
    float* friction_forces = cloth.mCloth->getFrictionForces();
    PxClothParticleData* particle_data = cloth.mCloth->lockParticleData(PxDataAccessFlag::eWRITABLE);

    // compute grip constraint force and modify particle positions
    for (int j = 0; j < rig_parts.size(); j++) {
        for (int i = 0; i < rig_parts[j].gripper_constraints.size(); i++) {
            PxVec3 change = rig_parts[j].gripper_constraints[i].second - particle_data->particles[rig_parts[j].gripper_constraints[i].first].pos;

            particle_data->particles[rig_parts[j].gripper_constraints[i].first].pos += change;

            change += particle_data->previousParticles[rig_parts[j].gripper_constraints[i].first].pos - rig_parts[j].gripper_previous[i];

            // add gravity compensation for mass scaled particles
            double iterDt = 1.0/cloth.mCloth->getSolverFrequency();
            change += (grip_massscale-1.0)/(grip_massscale) * (gScene->getGravity() * iterDt * iterDt);

            particle_data->previousParticles[rig_parts[j].gripper_constraints[i].first].pos = rig_parts[j].gripper_previous[i];

            friction_forces[rig_parts[j].gripper_constraints[i].first * 3] += change[0];
            friction_forces[rig_parts[j].gripper_constraints[i].first * 3+1] += change[1];
            friction_forces[rig_parts[j].gripper_constraints[i].first * 3+2] += change[2];
        }
    }

    // calculate forces
    for (int i = 0; i < cloth.mCloth->getNbParticles(); i++) {
        if (particle_data->particles[i].invWeight == 0) {
            continue;
        }
        PxVec3 total_force(contact_forces[i*3] + friction_forces[i*3], contact_forces[i*3+1] + friction_forces[i*3+1], contact_forces[i*3+2] + friction_forces[i*3+2]);

        total_force *= cloth.mCloth->getSolverFrequency() * 1/gTimeStep;
        total_force *= cloth.cloth_mesh.particles[i].mass;
        for (int j = 0; j < rig_parts.size(); j++) {
            rig_parts[j].updateContactForce(particle_data->particles[i].pos, total_force);
        }

        sphereManager->updateContactForce(particle_data->particles[i].pos, total_force);
    }
    particle_data->unlock();

    //cout << "eee\n";
    //cout << rig_parts[1].total_force.x << endl;
    //cout << rig_parts[1].total_torque.x << " " << rig_parts[1].total_torque.y << " " << rig_parts[1].total_torque.z << endl;*/
}

void gownSimulator::identifyGrippedParticles() {
    PxClothParticleData* particle_data = cloth.mCloth->lockParticleData(PxDataAccessFlag::eWRITABLE);
    PxBounds3 bound1 = rig_parts[1].components[0]->getWorldBounds();
    PxBounds3 bound2 = rig_parts[1].components[1]->getWorldBounds();
    for (int i = 0; i < cloth.cloth_mesh.particles.size(); i++) {
        if (bound1.contains(particle_data->particles[i].pos) && bound2.contains(particle_data->particles[i].pos)) {
            rig_parts[1].gripper_constraints.push_back(make_pair(i, particle_data->particles[i].pos));
            rig_parts[1].gripper_previous.push_back(particle_data->particles[i].pos);
            particle_data->particles[i].invWeight /= grip_massscale;
            particle_data->previousParticles[i].invWeight /= grip_massscale;
            cloth.cloth_mesh.particles[i].mass *= grip_massscale;
            //cout << i << ", ";
        }
    }
    cout << endl;

    //rig_parts[1].gripper_constraints.push_back(make_pair(0, particle_data->particles[0].pos));

    particle_data->unlock();

    cloth.mCloth->setTetherConfig(PxClothTetherConfig(1, 1));
    //cloth.mCloth->setSelfCollisionDistance(0.01f);
    //cloth.mCloth->setSelfCollisionStiffness(1.0f);

    cloth_solve_iteration = cloth_solve_iteration_aftergrip;

    gScene->removeActor(*rig_parts[1].components[0]);
    gScene->removeActor(*rig_parts[1].components[1]);
}

// set parameter
void gownSimulator::setParameter(VectorXd param) {
  cloth.setParameter(param.segment(0, (long) param.size() - (long) parameter_size));
    //cloth_solve_iteration_aftergrip = param(param.size()-1);
}




