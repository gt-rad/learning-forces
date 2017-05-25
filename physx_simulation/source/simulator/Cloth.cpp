//
//  Cloth.cpp
//  physx_test
//
//  Created by YuWenhao on 10/1/15.
//  Adapted by Zackory Erickson
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#include "Cloth.h"
#include "MeshReader.h"
#include <iostream>

using namespace std;
using namespace physx;
using namespace Eigen;

Cloth::~Cloth() {
    //mCloth->release();
}

void Cloth::createCloth(PxPhysics* physx, PxTransform transform, PxMeshScale scale) {
    // create regular mesh
    PxU32 numTriangles = cloth_mesh.faces.size();
    // minimal distance between particles, for self collision
    double min_dist = 100000;
    double max_dist = 0;
    for (int i = 0; i < cloth_mesh.faces.size(); i++) {
        for (int j = 0; j < 3; j++) {
            if ((cloth_mesh.faces[i].particles[j]->pos - cloth_mesh.faces[i].particles[(j+1)%3]->pos).norm() < min_dist) {
                min_dist = (cloth_mesh.faces[i].particles[j]->pos - cloth_mesh.faces[i].particles[(j+1)%3]->pos).norm();
            }
            if ((cloth_mesh.faces[i].particles[j]->pos - cloth_mesh.faces[i].particles[(j+1)%3]->pos).norm() > max_dist) {
                max_dist = (cloth_mesh.faces[i].particles[j]->pos - cloth_mesh.faces[i].particles[(j+1)%3]->pos).norm();
            }
        }
    }
    
    int numParticles = cloth_mesh.particles.size();
    
    // create cloth particles
    PxClothParticle* particles = new PxClothParticle[numParticles];
    PxVec4* restPositions = new PxVec4[numParticles];
    
    // create quads
    PxU32* triangles = new PxU32[3*numTriangles];
    int mId=0;
    {
        //static const int rawlist[] = {};
        static const int rawlist[] = {111, 112, 113, 114, 115, 415, 416, 417, 418, 560, 600, 759, 957, 1256, 1269, 1304, 1353, 1525, 1552, 2090, 2162, 2481, 2531, 2598, 2704, 2812, 2854, 3026, 3279, 3882};
        // static const int rawlist[] = {115, 116, 118, 119, 120, 121, 122, 123, 492, 560, 600, 1052, 1256, 1269, 1353, 1525, 2481, 2531, 2558, 2812, 2884, 3269, 3921, 3983, 4506, 4509, 4643, 4646, 4647, 4715, 5016, 5434, 6224, 6638, 6639, 6669, 6671, 6900, 7043, 7090, 7127, 7138, 7470, 7808, 8149, 8150, 8293, 8294, 8346, 8406, 8623, 8771, 8772, 8994, 9087, 9088, 9269, 9456};
        Eigen::Vector3d centPos(0, 0, 0);
        for (auto pId : rawlist) {
            centPos = centPos + cloth_mesh.particles[pId].pos;
        }
        centPos /= sizeof(rawlist) / sizeof(rawlist[0]);
        double mdist = 10000;
        for (auto iParticle : cloth_mesh.particles) {
            if ((iParticle.pos - centPos).norm() < mdist) {
                mId = iParticle.index;
                mdist = (iParticle.pos - centPos).norm();
            }
        }
    }
    std::cout << "tether anchor: " << mId << std::endl;
    vector<int> grip_list;
    grip_list.push_back(mId);
    //grip_list.clear();
    vector<double> temp_inv_weight;
    
    PxClothParticle* pIt = particles;
    PxVec4* posIt = restPositions;
    for(PxU32 i=0; i<numParticles; ++i, ++pIt, ++posIt)
    {
        pIt->invWeight = 1.0 / cloth_mesh.particles[i].mass;
        /*if (i > 120) {
         pIt->invWeight = 0.0;
         cloth_mesh.particles[i].mass = 10000000;
         initial_mesh.particles[i].mass = 10000000;
         }*/
        pIt->pos = PxVec3(cloth_mesh.particles[i].pos(0), cloth_mesh.particles[i].pos(1), cloth_mesh.particles[i].pos(2));
        
        *posIt = PxVec4(pIt->pos.x, pIt->pos.y, pIt->pos.z, 1);
        
        if (find(grip_list.begin(), grip_list.end(), i) != grip_list.end()) {
            temp_inv_weight.push_back(pIt->invWeight);
            //pIt->invWeight = 0;
        }
        
        //if (pIt->pos[1] > 15)
        //if (pIt->pos[2] < 2)
        //  pIt->invWeight = 0;
    }
    
    PxU32* iIt = triangles;
    for(PxU32 i=0; i<cloth_mesh.faces.size(); ++i)
    {
        *iIt++ = cloth_mesh.faces[i].particles[0]->index;
        *iIt++ = cloth_mesh.faces[i].particles[1]->index;
        *iIt++ = cloth_mesh.faces[i].particles[2]->index;
    }
    
    // create fabric from mesh
    PxClothMeshDesc meshDesc;
    meshDesc.points.count = numParticles;
    meshDesc.points.stride = sizeof(PxClothParticle);
    meshDesc.points.data = particles;
    
    meshDesc.invMasses.count = numParticles;
    meshDesc.invMasses.stride = sizeof(PxClothParticle);
    meshDesc.invMasses.data = &particles->invWeight;
    
    meshDesc.triangles.count = numTriangles;
    meshDesc.triangles.stride = 3*sizeof(PxU32);
    meshDesc.triangles.data = triangles;
    
    // cook fabric
    PxClothFabric* fabric = PxClothFabricCreate(*physx, meshDesc, PxVec3(0, 1, 0), true);
    
    fabric->scaleRestlengths(1);
    
    // create cloth
    mCloth = physx->createCloth(transform, *fabric, particles, PxClothFlags(0));
    
    // restore invWeight of the particles for tether anchor
    int tmp = 0;
    pIt = particles;
    PxClothParticleData* particle_data = mCloth->lockParticleData(PxDataAccessFlag::eWRITABLE);
    for(PxU32 i=0; i<numParticles; ++i, ++pIt)
    {
        if (find(grip_list.begin(), grip_list.end(), i) != grip_list.end()) {
            particle_data->particles[i].invWeight = temp_inv_weight[tmp++];
            particle_data->previousParticles[i].invWeight = particle_data->particles[i].invWeight;
        }
    }
    particle_data->unlock();
    
    mCloth->setStretchConfig(PxClothFabricPhaseType::eVERTICAL,PxClothStretchConfig(vstretch_stiff[0], vstretch_stiff[1], vstretch_stiff[2], vstretch_stiff[3]));
    mCloth->setStretchConfig(PxClothFabricPhaseType::eHORIZONTAL, PxClothStretchConfig(hstretch_stiff[0], hstretch_stiff[1], hstretch_stiff[2], hstretch_stiff[3]));
    mCloth->setStretchConfig(PxClothFabricPhaseType::eSHEARING,PxClothStretchConfig(shear_stiff[0], shear_stiff[1], shear_stiff[2], shear_stiff[3]));
    mCloth->setStretchConfig(PxClothFabricPhaseType::eBENDING, PxClothStretchConfig(bend_stiff[0], bend_stiff[1], bend_stiff[2], bend_stiff[3]));
    mCloth->setTetherConfig(PxClothTetherConfig(1, 1));
    mCloth->setFrictionCoefficient(friction);
    mCloth->setSelfFrictionCoefficient(self_friction);
    mCloth->setDampingCoefficient(PxVec3(damping[0], damping[1], damping[2]));
    cout << stiffpower << endl;
    mCloth->setStiffnessPower(stiffpower);
    // setup virtual points
    static PxVec3 weights[] =
    {
        PxVec3(1.0f / 3, 1.0f / 3, 1.0f / 3), // center point
        PxVec3(4.0f / 6, 1.0f / 6, 1.0f / 6), // off-center point
        PxVec3(0.5f, 0.5f, 0.0f),    // edge point
    };
    PxU32 numFaces = meshDesc.triangles.count;
    cout << numFaces << endl;
    PxU32* mtriangles = (PxU32*)meshDesc.triangles.data;
    PxU32* indices = new PxU32[7*4*numFaces];
    for (PxU32 i = 0, *it = indices; i < numFaces; i++)
    {
        PxU32* itriangle = (PxU32*)mtriangles;
        PxU32 v0 = itriangle[0];
        PxU32 v1 = itriangle[1];
        PxU32 v2 = itriangle[2];
        
        // center
        *it++ = v0; *it++ = v1; *it++ = v2; *it++ = 0;
        
        // off centers
        *it++ = v0; *it++ = v1; *it++ = v2; *it++ = 1;
        *it++ = v1; *it++ = v2; *it++ = v0; *it++ = 1;
        *it++ = v2; *it++ = v0; *it++ = v1; *it++ = 1;
        
        // edge points
        *it++ = v0; *it++ = v1; *it++ = v2; *it++ = 2;
        *it++ = v1; *it++ = v2; *it++ = v0; *it++ = 2;
        *it++ = v2; *it++ = v0; *it++ = v1; *it++ = 2;
        
        itriangle += 3;
    }
    
    //mCloth->setVirtualParticles(numFaces*7, indices, 3, weights);
    delete[] indices;
    
    mCloth->setRestPositions(restPositions);
    delete[] restPositions;
    
    fabric->release();
    delete[] particles;
    delete[] triangles;
    
    mCloth->setSolverFrequency(300.0);
    
    mCloth->setStiffnessFrequency(300);
    
    mCloth->setClothFlag(PxClothFlag::eSWEPT_CONTACT, true);
    mCloth->setClothFlag(PxClothFlag::eSCENE_COLLISION, true);
    
    //mCloth->setSelfCollisionDistance(min_dist - 0.0000000000000001f);
    cout << "min dist: " << min_dist << endl;
    cout << "max dist: " << max_dist << endl;
    mCloth->setSelfCollisionDistance(self_collision_distance);
    mCloth->setSelfCollisionStiffness(self_collision_stiff);
    
    cout << "tether number: " << fabric->getNbTethers() << " " << mCloth->getFabric()->getNbTethers() << endl;
}

void Cloth::loadMesh(string filename, Eigen::Vector3d translation, Eigen::Matrix3d rotation, Eigen::Vector3d scale) {
    MeshReader reader;
    reader.readObjMesh(cloth_mesh, filename, translation, rotation, scale);
    cloth_mesh.updateArea();
    // update the mass
    double total_mass = 0;
    for (int i = 0; i < cloth_mesh.particles.size(); i++) {
        cloth_mesh.particles[i].mass = 0;
    }
    for (int i = 0; i < cloth_mesh.faces.size(); i++) {
        total_mass += cloth_mesh.faces[i].area * density;
    }
    double particle_mass = total_mass / cloth_mesh.particles.size();
    for (int i = 0; i < cloth_mesh.particles.size(); i++) {
        cloth_mesh.particles[i].mass = particle_mass;
    }
    
    initial_mesh.copy(cloth_mesh);
    cout << "particle number: " << cloth_mesh.particles.size() << endl;
    cout << "total mass: " << total_mass << endl;
}

void Cloth::resetCloth() {
    cloth_mesh.copy(initial_mesh);
    max_stretch = -1;
}

void Cloth::updateMesh() {
    PxClothParticleData* particle_data = mCloth->lockParticleData();
    
    for (int i = 0; i < cloth_mesh.particles.size(); i++) {
        cloth_mesh.particles[i].pos = Eigen::Vector3d(particle_data->particles[i].pos[0], particle_data->particles[i].pos[1], particle_data->particles[i].pos[2]);
    }
    
    particle_data->unlock();
    double ls = -1;
    for (int i = 0; i < cloth_mesh.faces.size(); i++) {
        for (int j = 0; j < 3; j++) {
            double cur_len = (cloth_mesh.faces[i].particles[j]->pos-cloth_mesh.faces[i].particles[(j+1)%3]->pos).norm();
            double rest_len = (initial_mesh.faces[i].particles[j]->pos-initial_mesh.faces[i].particles[(j+1)%3]->pos).norm();
            
            if (cur_len / rest_len > ls) {
                ls = cur_len/rest_len;
            }
        }
    }
    
    max_stretch = ls;
    //cout << "max stretch: " << max_stretch << endl;
}

void Cloth::setParameter(VectorXd parameters) {
    if (parameters.size() != parameter_size) {
        cout << "parameter size wrong" << endl;
        return;
    }
    
    VectorXd min, max;
    getParameterBound(min, max);
    for (int i = 0; i < parameter_size; i++) {
        if (parameters(i) > max(i)) {
            parameters(i) = max(i);
        } else if (parameters(i) < min(i)) {
            parameters(i) = min(i);
        }
    }
    
    int param_index = 0;
    
    vstretch_stiff[0] = hstretch_stiff[0] = parameters[0];
    shear_stiff[0] = bend_stiff[0] = parameters[1];
    friction = parameters[2];
    self_friction = parameters[3];
    self_collision_distance = parameters[4];
    mCloth->setStretchConfig(PxClothFabricPhaseType::eVERTICAL,PxClothStretchConfig(vstretch_stiff[0], vstretch_stiff[1], vstretch_stiff[2], vstretch_stiff[3]));
    mCloth->setStretchConfig(PxClothFabricPhaseType::eHORIZONTAL, PxClothStretchConfig(hstretch_stiff[0], hstretch_stiff[1], hstretch_stiff[2], hstretch_stiff[3]));
    mCloth->setStretchConfig(PxClothFabricPhaseType::eSHEARING,PxClothStretchConfig(shear_stiff[0], shear_stiff[1], shear_stiff[2], shear_stiff[3]));
    mCloth->setStretchConfig(PxClothFabricPhaseType::eBENDING, PxClothStretchConfig(bend_stiff[0], bend_stiff[1], bend_stiff[2], bend_stiff[3]));
    mCloth->setFrictionCoefficient(friction);
    mCloth->setSelfFrictionCoefficient(self_friction);
    mCloth->setFrictionCoefficient(friction);
    mCloth->setSelfCollisionDistance(self_collision_distance);
}

void Cloth::setParameters() {
    mCloth->setStretchConfig(PxClothFabricPhaseType::eVERTICAL,PxClothStretchConfig(vstretch_stiff[0], vstretch_stiff[1], vstretch_stiff[2], vstretch_stiff[3]));
    mCloth->setStretchConfig(PxClothFabricPhaseType::eHORIZONTAL, PxClothStretchConfig(hstretch_stiff[0], hstretch_stiff[1], hstretch_stiff[2], hstretch_stiff[3]));
    mCloth->setStretchConfig(PxClothFabricPhaseType::eSHEARING,PxClothStretchConfig(shear_stiff[0], shear_stiff[1], shear_stiff[2], shear_stiff[3]));
    mCloth->setStretchConfig(PxClothFabricPhaseType::eBENDING, PxClothStretchConfig(bend_stiff[0], bend_stiff[1], bend_stiff[2], bend_stiff[3]));
    mCloth->setTetherConfig(PxClothTetherConfig(1, 1));
    mCloth->setFrictionCoefficient(friction);
    mCloth->setSelfFrictionCoefficient(self_friction);
    mCloth->setDampingCoefficient(PxVec3(damping[0], damping[1], damping[2]));
    
    mCloth->setSelfCollisionDistance(self_collision_distance);
    mCloth->setSelfCollisionStiffness(self_collision_stiff);
    mCloth->setStiffnessPower(stiffpower);
}

void Cloth::getParameterBound(VectorXd& min, VectorXd& max) {
    min.resize(parameter_size);
    max.resize(parameter_size);
    
    min.setZero();
    
    for (int i = 0; i < parameter_size; i++) {
        max(i) = 1;
    }
    min(parameter_size-1) = 0.005;
    max(parameter_size-1) = 0.08;
}

