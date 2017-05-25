//
//  main.cpp
//  evaluator
//
//  Created by YuWenhao on 2/4/16.
//  Copyright (c) 2016 YuWenhao. All rights reserved.
//

#include <iostream>

#include "utils/CppCommon.h"

#include "myUtils/ConfigManager.h"

#include "config.h"

// gown or sleeve
#include "simulator/gownSimulator.h"
//#include "Simulator.h"
#include "CMAES/gownData.h"
#include "CMAES/HapticObjective.h"
#include <PxPhysicsAPI.h>
#include <sstream>
#include <iomanip>
#include <boost/interprocess/ipc/message_queue.hpp>
using namespace boost::interprocess;

using namespace std;
using namespace physx;

// gown or sleeve
gownSimulator simulator;
HapticObjective simp_good;
HapticObjective simp_miss;
HapticObjective simp_caught;

int gProcessId = 0;

double good_height_perturbation = 0;
double miss_height_perturbation = 0;
double caught_height_perturbation = 0;

double good_horizontal_perturbation = 0;
double miss_horizontal_perturbation = 0;
double caught_horizontal_perturbation = 0;

void setSimulatorParameter(gownSimulator* simulator, const vector<double>& parameter, gownData* data) {
    int index = 0;
    double * lbound = new double[data->GetNumParameters()];
    double * ubound = new double[data->GetNumParameters()];
    data->GetParameterLowerBounds(lbound);
    data->GetParameterUpperBounds(ubound);
    if (data->useParameter(gownData::STRETCH)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        simulator->cloth.vstretch_stiff[0] = value;
        simulator->cloth.hstretch_stiff[0] = value;
        LOG(INFO) << "stretch " << value;
        index++;
    }
    if (data->useParameter(gownData::SHEAR)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        simulator->cloth.shear_stiff[0] = value;
        LOG(INFO) << "Shear " << parameter[index] << " " << value;
        index++;
    }
    if (data->useParameter(gownData::BEND)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        simulator->cloth.bend_stiff[0] = value;
        LOG(INFO) << "Bend " << parameter[index] << " " << value;
        index++;
    }
    if (data->useParameter(gownData::FRICTION)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "friction " << parameter[index] << " " << value;
        simulator->cloth.friction = value;
        index++;
    }
    if (data->useParameter(gownData::SELFCOLDIST)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Self col dist " << parameter[index] << " " << value;
        simulator->cloth.self_collision_distance = value;
        index++;
    }
    if (data->useParameter(gownData::SELFCOLFRICTION)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Self friction " << parameter[index] << " " << value;
        simulator->cloth.self_friction = value;
        index++;
    }
    if (data->useParameter(gownData::STIFFPOWER)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Stiffness Power " << parameter[index] << " " << (unsigned int)value;
        simulator->cloth.stiffpower = (unsigned int)value;
        index++;
    }
    if (data->useParameter(gownData::ITERATION)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Iteration " << parameter[index] << " " << value;
        simulator->cloth_solve_iteration_aftergrip = value;
        index++;
    }
    if (data->useParameter(gownData::WIND)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Wind " << parameter[index] << " " << value;
        simulator->random_wind.x = (value)/4;
        simulator->random_wind.z = (sqrt(1-value*value))/4;
        index++;
    }
    if (data->useParameter(gownData::ARM_HEIGHT_PERT_GOOD)) {
        good_height_perturbation = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Height Perturbation in good: " << parameter[index] << " " << good_height_perturbation;
        index++;
    }
    if (data->useParameter(gownData::ARM_HEIGHT_PERT_MISS)) {
        miss_height_perturbation = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Height Perturbation in miss: " << parameter[index] << " " << miss_height_perturbation;
        index++;
    }
    if (data->useParameter(gownData::ARM_HEIGHT_PERT_CAUG)) {
        caught_height_perturbation = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Height Perturbation in caught: " << parameter[index] << " " << caught_height_perturbation;
        index++;
    }
    if (data->useParameter(gownData::ARM_HORIZONTAL_PERT_GOOD)) {
        good_horizontal_perturbation = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Horizontal Perturbation in good: " << parameter[index] << " " << good_horizontal_perturbation;
        index++;
    }
    if (data->useParameter(gownData::ARM_HORIZONTAL_PERT_MISS)) {
        miss_horizontal_perturbation = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Horizontal Perturbation in miss: " << parameter[index] << " " << miss_horizontal_perturbation;
        index++;
    }
    if (data->useParameter(gownData::ARM_HORIZONTAL_PERT_CAUG)) {
        caught_horizontal_perturbation = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Horizontal Perturbation in caught: " << parameter[index] << " " << caught_horizontal_perturbation;
        index++;
    }
    if (data->useParameter(gownData::FIST_RADIUS)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Fist radius: " << parameter[index] << " " << value;
        index++;
        simulator->fist_radius = value;
    }
    if (data->useParameter(gownData::WRIST_RADIUS)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Wrist radius: " << parameter[index] << " " << value;
        index++;
        simulator->wrist_radius = value;
    }
    if (data->useParameter(gownData::ELBOW_RADIUS)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Elbow radius: " << parameter[index] << " " << value;
        index++;
        simulator->elbow_radius = value;
    }
    if (data->useParameter(gownData::SHOULDER_RADIUS)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Shoulder radius: " << parameter[index] << " " << value;
        index++;
        simulator->fist_radius = value;
    }
    if (data->useParameter(gownData::FOREARM_LENGTH)) {
        double value = parameter[index] * (ubound[index] - lbound[index]) + lbound[index];
        LOG(INFO) << "Forearm length: " << parameter[index] << " " << value;
        index++;
        simulator->forearm_length = value;
    }
    
    LOG(INFO) << "Forearm Length used: " << simulator->forearm_length;
    
    // reset the arm model
    PxSphereManager::GetSingleton()->clearSpheres();
    PxSphereManager* sphereManager = PxSphereManager::GetSingleton();
    sphereManager->addSphere(PxVec3(-simulator->fist_radius-0.1+simulator->arm_horizontal_perturb, simulator->arm_height, 0), simulator->fist_radius); // fist
    sphereManager->addSphere(PxVec3(-simulator->fist_radius-0.1+simulator->arm_horizontal_perturb, simulator->arm_height, 0), simulator->wrist_radius); // wrist
    sphereManager->addSphere(PxVec3(-simulator->forearm_length-simulator->elbow_radius-0.1+simulator->arm_horizontal_perturb, simulator->arm_height, 0), simulator->elbow_radius); // elbow
    sphereManager->addSphere(PxVec3(-simulator->forearm_length-simulator->elbow_radius-0.1+simulator->arm_horizontal_perturb, simulator->arm_height, -simulator->upperearm_length), simulator->shoulder_radius); // shoulder
    sphereManager->recordInitSpheres();
    
    LOG(INFO) << "pid: " << gProcessId;
    simulator->cloth.setParameters();
    
    delete []lbound;
    delete []ubound;
}

int main(int argc, char* argv[])
{
    // google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging((const char*)argv[0]);
    // Define logging flag
    FLAGS_alsologtostderr = true;
    FLAGS_minloglevel = google::INFO;
#ifndef _DEBUG
    FLAGS_log_dir = "./glog/";
#endif
    LOG(INFO) << "BioloidGP program begins...";
    
    
    //srand( (unsigned int) time (NULL) );
    
    // initialize the system and evaluation objects
    simulator.initialize();
    // gown or simp
    string config_data;
    DecoConfig::GetSingleton()->GetString("CMA", "GoodPath", config_data);
    simp_good.ReadData(string(PHYSX_ROOT_PATH)+config_data);
    DecoConfig::GetSingleton()->GetString("CMA", "MissPath", config_data);
    simp_miss.ReadData(string(PHYSX_ROOT_PATH)+config_data);
    if (DecoConfig::GetSingleton()->GetString("CMA", "CaughtPath", config_data)) {
        simp_caught.ReadData(string(PHYSX_ROOT_PATH)+config_data);
    }
    
    
    
    CMAData* cmaData = new gownData();
    if (argc == 2)
    {
        gProcessId = atoi(argv[1]);
    }
    else
    {
        LOG(INFO) << "Wrong argc setup\n";
        return 0;
    }
    
    int policyDim = cmaData->GetNumParameters();
    cout << "policy dimension: " << policyDim << endl;
    vector<vector<double> > parameters;
    while (true)
    {
        try {
            LOG(INFO) << gProcessId << "th process.";
            // creating a message queue
            char queueName[128];
            
            sprintf(queueName, "mqToSlave%d", gProcessId);
            message_queue mqIn(open_only, queueName);
            size_t recvd_size;
            unsigned int priority;
            double numEval;
            LOG(INFO) << "waiting to receive.";
            mqIn.receive((void*)&numEval, sizeof(double), recvd_size, priority);
            int numEvaluations = static_cast<int>(numEval + 0.5);
            if (numEvaluations <= 0) {
                LOG(INFO) << "Process " << gProcessId << " received negative evaluation number " << numEvaluations;
                break;
            }
            
            LOG(INFO) << "Parameter received.";
            parameters.resize(numEvaluations);
            for (int ithEvaluation = 0; ithEvaluation < numEvaluations; ++ithEvaluation)
            {
                parameters[ithEvaluation].resize(policyDim);
                
                for (int i = 0; i < policyDim; ++i)
                {
                    double p;
                    mqIn.receive((void*)&p, sizeof(double), recvd_size, priority);
                    parameters[ithEvaluation][i] = p;
                }
            }
            
            LOG(INFO) << "calculating. " << numEvaluations;
            
            vector<double> values;
            values.resize(numEvaluations, 0);
            
            for (int i = 0; i < numEvaluations; ++i)
            {
                double val = 0;
                
                setSimulatorParameter(&simulator, parameters[i], (gownData*)cmaData);

                // gown or simp
                double target_height = -0.13 + simulator.wrist_radius + 0.205 + good_height_perturbation;
                PxSphereManager* sphereManager = PxSphereManager::GetSingleton();
                simulator.reset();
                simulator.simulate(1);
                simulator.reset();
                
                
                 for (int c = 0; c < sphereManager->getNumSphere(); c++) {
                     sphereManager->translate(c, PxVec3(good_horizontal_perturbation-simulator.arm_horizontal_perturb, target_height - simulator.arm_height, 0));
                 }
                sphereManager->updateCapsules(simulator.cloth.mCloth);
                cout << "Used height: " << target_height << endl;

                while (simulator.simulated_step >= -1) {
                    simulator.simulate(3);
                }
                val += simp_good.evalObjective(&simulator);
                LOG(INFO) << "Good sim finished: " << gProcessId << " " << val;
                
                target_height = -0.13 + simulator.wrist_radius + miss_height_perturbation;
                simulator.reset();
                simulator.simulate(1);
                simulator.reset();
                for (int c = 0; c < sphereManager->getNumSphere(); c++) {
                    sphereManager->translate(c, PxVec3(miss_horizontal_perturbation-simulator.arm_horizontal_perturb, target_height - simulator.arm_height, 0));
                }
                sphereManager->updateCapsules(simulator.cloth.mCloth);
                while (simulator.simulated_step >= -1) {
                    simulator.simulate(3);
                }
                
                val += simp_miss.evalObjective(&simulator);
                LOG(INFO) << "Miss sim finished: " << gProcessId << " " << val;
                
                if (simp_caught.exp_time.size() == 0) { // no caught data is provided
                    target_height = -0.13 + simulator.wrist_radius + 0.205/5*2 + caught_height_perturbation;
                    simulator.reset();
                    simulator.simulate(1);
                    simulator.reset();
                    for (int c = 0; c < sphereManager->getNumSphere(); c++) {
                        sphereManager->translate(c, PxVec3(caught_horizontal_perturbation-simulator.arm_horizontal_perturb, target_height - simulator.arm_height, 0));
                    }
                    sphereManager->updateCapsules(simulator.cloth.mCloth);
                    
                    while (simulator.simulated_step >= -1) {
                        simulator.simulate(3);
                    }
                    
                    val += simp_caught.evalObjective(&simulator);
                    LOG(INFO) << "Caught sim finished: " << gProcessId << " " << val;
                }
                
                values[i] = val;
            }
            
            LOG(INFO) << "ready to send.";
            sprintf(queueName, "mqFromSlave%d", gProcessId);
            message_queue mqOut(open_only, queueName);
            for (int i = 0; i < numEvaluations; ++i)
            {
                LOG(INFO) << "Obj val: " << values[i];
                mqOut.send(&(values[i]), sizeof(double), 0);
            }
            LOG(INFO) << "done.";
            
        }
        catch (interprocess_exception& e) {
            std::cout << e.what() << std::endl;
        }
    }
    
    LOG(INFO) << "Exited Process " << gProcessId;
    
    delete cmaData;
    
    return 0;
}

