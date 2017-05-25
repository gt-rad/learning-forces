//
//  Mesh.h
//  MeshRelaxation
//
//  Created by YuWenhao on 9/25/15.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef __MeshRelaxation__Mesh__
#define __MeshRelaxation__Mesh__

#include <stdio.h>
#include <vector>
#undef Success
#include <Eigen/Dense>

struct Particle {
    int index;
    Eigen::Vector3d pos;
    
    Eigen::Vector3d normal;
    
    double mass;
};

struct Edge {
    Edge() {rest_length = 0;}
    Edge(Particle* p1, Particle* p2, double rest_length);
    Particle* particles[2];
    
    int index[2];
    
    double rest_length;
};

struct Triangle {
    Triangle() {}
    Triangle(Particle* p1, Particle* p2, Particle* p3);
    std::vector<Particle*> particles;
    
    double area = 0;
    
    int index[3];
    
    double calcVolume();
    
    void calcArea();
};

struct Mesh {
    Mesh(){}
    Mesh(const Mesh&);
    
    std::vector<Particle> particles;
    std::vector<Edge> edges;
    std::vector<Triangle> faces;
    
    // generate constraints from faces
    // assume triangle faces
    void generateConstraints();
    
    void copy(const Mesh&);
    
    void updateNorm();
    
    void updateArea() {
        for (int i = 0; i < faces.size(); i++) {
            faces[i].calcArea();
        }
    }
    
    void cleanMesh() {
        faces.clear();
        edges.clear();
        particles.clear();
    }
    
    void backupIndex();
    
    void restoreTopology();
};

// calculate constraint value
double calcTotalConstraint(Mesh&);


#endif /* defined(__MeshRelaxation__Mesh__) */
