//
//  PxSphereManager.h
//  physx_test
//
//  Created by YuWenhao on 2/24/16.
//  Adapted by Zackory Erickson
//  Copyright (c) 2016 YuWenhao. All rights reserved.
//

#ifndef __physx_test__PxSphereManager__
#define __physx_test__PxSphereManager__

#include <stdio.h>
#include <vector>
#include <tuple>
#include <PxPhysicsAPI.h>

class PxSphereManager {
public:
    PxSphereManager() {
        mSphereManager = NULL;
    }
    
    static PxSphereManager* GetSingleton();
    static void DestroySingleton();
    
    void updateContactForce(physx::PxVec3 orig, physx::PxVec3 force, int maxSphereIndex = -1);
    
    void clearForceMap();
    
    void recordForceMap();

    std::vector<physx::PxVec4> getForceMap() {
        return forcemap;
    }

    std::vector<std::vector<double>> getFullForceMap() {
        return fullForcemap;
    }

    std::vector<physx::PxVec4> getSpheres();
    
    void addSphere(physx::PxVec3 center, double radius);
    
    void addCapsule(uint32_t first, uint32_t second);
    
    void setUpCapsules(physx::PxCloth*);
    
    void updateCapsules(physx::PxCloth*);
        
    void getSphereData(std::vector<physx::PxClothCollisionSphere>&, std::vector<std::pair<uint32_t, uint32_t> >&);
    
    void translate(int, physx::PxVec3);
    
    void rotate(int, physx::PxReal, physx::PxVec3);
    
    void rotateAround(int, physx::PxReal, physx::PxVec3, physx::PxVec3);
    
    int getNumSphere() {
        return mNumSphere;
    }
    
    void recordInitSpheres();
    
    void resetPos();
    
    void clearSpheres();
private:
    static PxSphereManager* mSphereManager;
    
    physx::PxClothCollisionSphere spheres[32];
    int mNumSphere = 0;
    physx::PxClothCollisionSphere initspheres[32];
    
    std::vector<std::tuple<uint32_t, uint32_t, float>> capsules;

    std::vector<physx::PxVec4> forcemap;
    std::vector<std::vector<double>> fullForcemap;
    // std::vector<std::vector<physx::PxVec4>> forcemapSpheres;
    // std::vector<std::vector<physx::PxVec4>> forcemapCapsules;
};

#endif /* defined(__physx_test__PxSphereManager__) */
