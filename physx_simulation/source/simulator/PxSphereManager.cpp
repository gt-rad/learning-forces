//
//  PxSphereManager.cpp
//  physx_test
//
//  Created by YuWenhao on 2/24/16.
//  Adapted by Zackory Erickson
//  Copyright (c) 2016 YuWenhao. All rights reserved.
//

#include <iostream>
#include "PxSphereManager.h"

using namespace physx;
using namespace std;

PxSphereManager* PxSphereManager::mSphereManager = NULL;

PxSphereManager* PxSphereManager::GetSingleton() {
    if (!mSphereManager)
    {
        mSphereManager = new PxSphereManager();
    }
    return mSphereManager;
}

void PxSphereManager::DestroySingleton() {
    if (mSphereManager) {
        delete mSphereManager;
    }
}



void PxSphereManager::updateContactForce(PxVec3 orig, PxVec3 force, int maxSphereIndex) {
    // Make sure none of the values are NaN
    if (!orig.isFinite() || !force.isFinite() || force.magnitude() == 0)
        return;

    // Determine if this contact force is on a sphere (check each sphere)
    // maxSphereIndex limits the number of spheres we update contact forces for.
    for (int i = 0; i < mNumSphere; i++) {
        if (maxSphereIndex >= 0 && i > maxSphereIndex)
            break;
        
        // Determine the Euclidean norm (distance) between the force point and the center of the sphere
        float distance = (spheres[i].pos - orig).magnitude();
        if (distance < spheres[i].radius) {
            // The force point is within the sphere. Add the point and force magnitude to our forcemap
            forcemap.push_back(PxVec4(orig, force.magnitude()));
            fullForcemap.push_back((vector<double>){orig.x, orig.y, orig.z, force.x, force.y, force.z});
            // cout << "force magnitude: " << force.magnitude() << " | force origin: " << orig.x << ", " << orig.y << ", " << orig.z << endl;
            return;
        }
    }

    // Determine if this contact force is on a capsule (treat them as cylinders)
    for (int i = 0; i < capsules.size(); i++) {
        // Vector from one endpoint of cylinder to the other (first capsule sphere at origin)
        PxVec3 cylVec = spheres[std::get<1>(capsules[i])].pos - spheres[std::get<0>(capsules[i])].pos;
        float lengthSqr = cylVec.magnitudeSquared();
        float radiusSqr = PxPow(get<2>(capsules[i]), 2);
        // Vector from force point to first capsule sphere
        PxVec3 pointVec = orig - spheres[get<0>(capsules[i])].pos;
        // Take dot product of cylVec and pointVec
        float dotProd = cylVec.dot(pointVec);
        // Determine if point is beyond either of the endcaps using the dot product
        if (dotProd < 0 || dotProd > lengthSqr)
            continue;
        // Distance squared to the cylinder axis
        float distSqr = pointVec.dot(pointVec) - PxPow(dotProd, 2) / lengthSqr;
        // Check if point is outside of cylinder radius
        if (distSqr > radiusSqr)
            continue;
        // Force point is within the cylinder radius
        forcemap.push_back(PxVec4(orig, force.magnitude()));
        // cout << "force magnitude: " << force.magnitude() << " | force origin: " << orig.x << ", " << orig.y << ", " << orig.z << endl;
        return;
    }
}

void PxSphereManager::clearForceMap() {
    // Clear out the forcemap
    // for (int i = 0; i < forcemapSpheres.size(); i++)
        // forcemapSpheres[i].clear();
    forcemap.clear();
}

void PxSphereManager::recordForceMap() {
    // TODO: If needed
}

vector<PxVec4> PxSphereManager::getSpheres() {
    vector<PxVec4> sphereData(mNumSphere);
    for (int i = 0; i < mNumSphere; i++)
        sphereData[i] = PxVec4(spheres[i].pos, spheres[i].radius);
    return sphereData;
}


void PxSphereManager::addSphere(physx::PxVec3 center, double radius) {
    if (mNumSphere < 32) {
        spheres[mNumSphere++] = PxClothCollisionSphere(center, radius);
        // forcemaps.push_back(vector<PxVec4>)
    }
}

void PxSphereManager::addCapsule(uint32_t first, uint32_t second) {
    // Determine the largest radius between the spheres to use as an approximate radius for the capsule (when treated as a cylinder)
    float maxRadius = max(spheres[first].radius, spheres[second].radius);
    // float avgRadius = (spheres[first].radius + spheres[second].radius) / 2.0f;
    capsules.push_back(make_tuple(first, second, maxRadius));
}

void PxSphereManager::setUpCapsules(PxCloth* cloth) {
    cloth->setCollisionSpheres(spheres, mNumSphere);
    
    for (int i = 0; i < capsules.size(); i++) {
        cloth->addCollisionCapsule(std::get<0>(capsules[i]), std::get<1>(capsules[i]));
    }
}

void PxSphereManager::updateCapsules(PxCloth* cloth) {
    cloth->setCollisionSpheres(spheres, mNumSphere);
}

void PxSphereManager::getSphereData(vector<PxClothCollisionSphere>& sp, vector<pair<uint32_t, uint32_t> >& cp) {
    sp.clear();
    cp.clear();
    
    for (int i = 0; i < mNumSphere; i++) {
        sp.push_back(PxClothCollisionSphere(spheres[i].pos, spheres[i].radius));
    }
    for (int i = 0; i < capsules.size(); i++) {
        cp.push_back(make_pair(std::get<0>(capsules[i]), std::get<1>(capsules[i])));
    }
}


void PxSphereManager::translate(int ind, PxVec3 vec) {
    spheres[ind].pos += vec;
}

void PxSphereManager::rotate(int ind, PxReal angle, PxVec3 axis) {
    spheres[ind].pos = PxQuat(angle, axis.getNormalized()).getNormalized().rotate(spheres[ind].pos);
}

void PxSphereManager::rotateAround(int ind, PxReal angle, PxVec3 axis, PxVec3 center) {
    spheres[ind].pos = PxQuat(angle, axis.getNormalized()).getNormalized().rotate(spheres[ind].pos - center) + center;
}

void PxSphereManager::recordInitSpheres() {
    for (int i = 0 ; i < mNumSphere; i++) {
        initspheres[i].pos = spheres[i].pos;
        initspheres[i].radius = spheres[i].radius;
    }
}

void PxSphereManager::resetPos() {
    for (int i = 0 ; i < mNumSphere; i++) {
        spheres[i].pos = initspheres[i].pos;
        spheres[i].radius = initspheres[i].radius;
    }
}

void PxSphereManager::clearSpheres() {
    mNumSphere = 0;
    clearForceMap();
}

