//
//  Mesh.cpp
//  MeshRelaxation
//
//  Created by YuWenhao on 9/25/15.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#include "Mesh.h"
#include <vector>
#include <map>
#include <iostream>

using namespace std;
using namespace Eigen;

Edge::Edge(Particle* p1, Particle* p2, double r_length) {
    particles[0] = p1;
    particles[1] = p2;
    rest_length = r_length;
}

Triangle::Triangle(Particle* p1, Particle* p2, Particle* p3) {
    particles.push_back(p1);
    particles.push_back(p2);
    particles.push_back(p3);
}

void Mesh::generateConstraints() {
    vector<map<int, int> > connection_matrix(particles.size());
    for (int f = 0; f < faces.size(); f++) {
        int ind1 = faces[f].particles[0]->index,
        ind2 = faces[f].particles[1]->index,
        ind3 = faces[f].particles[2]->index;
        
        connection_matrix[ind1][ind2] = 1;
        connection_matrix[ind2][ind1] = 1;
        connection_matrix[ind1][ind3] = 1;
        connection_matrix[ind3][ind1] = 1;
        connection_matrix[ind2][ind3] = 1;
        connection_matrix[ind3][ind2] = 1;
    }
    
    for (int p = 0; p < particles.size(); p++) {
        map<int, int>::iterator iter = connection_matrix[p].begin();
        for (; iter != connection_matrix[p].end(); iter++) {
            // one ring edges
            if (iter->second == 1) {
                // create edge
                Edge edge(&particles[p], &particles[iter->first], (particles[iter->first].pos - particles[p].pos).norm());
                edges.push_back(edge);
                
                connection_matrix[iter->first][p] = 0;
            }
        }
    }
    
    // edge-face map
    map<pair<int, int>, pair<int, int> > edge_face_map;
    for (int f = 0; f < faces.size(); f++) {
        int ind1 = faces[f].particles[0]->index,
        ind2 = faces[f].particles[1]->index,
        ind3 = faces[f].particles[2]->index;
        
        if (edge_face_map[make_pair(min(ind1, ind2), max(ind1, ind2))].second == -1) {
            edge_face_map[make_pair(min(ind1, ind2), max(ind1, ind2))].second = f;
        } else {
            edge_face_map[make_pair(min(ind1, ind2), max(ind1, ind2))].first = f;
            edge_face_map[make_pair(min(ind1, ind2), max(ind1, ind2))].second = -1;
        }
        
        if (edge_face_map[make_pair(min(ind1, ind3), max(ind1, ind3))].second == -1) {
            edge_face_map[make_pair(min(ind1, ind3), max(ind1, ind3))].second = f;
        } else {
            edge_face_map[make_pair(min(ind1, ind3), max(ind1, ind3))].first = f;
            edge_face_map[make_pair(min(ind1, ind3), max(ind1, ind3))].second = -1;
        }
        
        if (edge_face_map[make_pair(min(ind3, ind2), max(ind3, ind2))].second == -1) {
            edge_face_map[make_pair(min(ind3, ind2), max(ind3, ind2))].second = f;
        } else {
            edge_face_map[make_pair(min(ind3, ind2), max(ind3, ind2))].first = f;
            edge_face_map[make_pair(min(ind3, ind2), max(ind3, ind2))].second = -1;
        }
    }
    
    map<pair<int, int>, pair<int, int> >::iterator ef_iter = edge_face_map.begin();
    for (; ef_iter != edge_face_map.end(); ef_iter++) {
        if (ef_iter->second.second == -1) {
            continue;
        }
        
        const Triangle& face1 = faces[ef_iter->second.first];
        const Triangle& face2 = faces[ef_iter->second.second];
        
        Particle* shared1, *shared2, *oppo1 = NULL, *oppo2 = NULL;
        
        VectorXd norm1 = ((face1.particles[1]->pos - face1.particles[0]->pos).cross(face1.particles[2]->pos - face1.particles[0]->pos)).normalized(),
        norm2 = ((face2.particles[1]->pos - face2.particles[0]->pos).cross(face2.particles[2]->pos - face2.particles[0]->pos)).normalized();
        
        shared1 = &particles[ef_iter->first.first];
        shared2 = &particles[ef_iter->first.second];
        
        for (int i = 0; i < 3; i++) {
            if (face1.particles[i]->index != ef_iter->first.first &&
                face1.particles[i]->index != ef_iter->first.second) {
                oppo1 = &particles[face1.particles[i]->index];
            }
            
            if (face2.particles[i]->index != ef_iter->first.first &&
                face2.particles[i]->index != ef_iter->first.second) {
                oppo2 = &particles[face2.particles[i]->index];
            }
        }
        
        double length_ratio = (oppo2->pos - oppo1->pos).norm() / ((shared1->pos - shared2->pos).norm() + 0.000001);
        double dot_ratio = (oppo2->pos - oppo1->pos).normalized().dot((shared1->pos - shared2->pos).normalized());
        double dihedral_ratio = norm1.dot(norm2);
        
        if (length_ratio < 0.9 || length_ratio > 1/0.9) {
            continue;
        }
        
        if (fabs(dot_ratio) > 0.5) {
            continue;
        }
        
        if (dihedral_ratio < 0.9) {
            continue;
        }
        
        edges.push_back(Edge(oppo1, oppo2, (oppo1->pos - oppo2->pos).norm()));
    }
}

Mesh::Mesh(const Mesh& m) {
    cleanMesh();
    particles = m.particles;
    edges = m.edges;
    faces = m.faces;
    
    backupIndex();
    restoreTopology();
}

void Mesh::copy(const Mesh& m) {
    cleanMesh();
    particles = m.particles;
    edges = m.edges;
    faces = m.faces;
    
    backupIndex();
    restoreTopology();
}

void Mesh::updateNorm() {
    for (int i = 0; i < particles.size(); i++) {
        particles[i].normal.setZero();
    }
    
    for (int f = 0; f < faces.size(); f++) {
        Vector3d norm = (faces[f].particles[1]->pos - faces[f].particles[0]->pos).cross(faces[f].particles[2]->pos - faces[f].particles[0]->pos).normalized();
        particles[faces[f].particles[0]->index].normal += norm;
        particles[faces[f].particles[1]->index].normal += norm;
        particles[faces[f].particles[2]->index].normal += norm;
    }
    
    for (int i = 0; i < particles.size(); i++) {
        particles[i].normal.normalize();
    }
}

double Triangle::calcVolume() {
    Vector3d pos1 = particles[0]->pos,
        pos2 = particles[1]->pos,
    pos3 = particles[2]->pos;
    
    return 0.5 * pos3.dot(pos1.cross(pos2));
}

double calcTotalConstraint(Mesh& mesh) {
    double total_constraint = 0;
    for (int e = 0; e < mesh.edges.size(); e++) {
        double current_length = (mesh.edges[e].particles[0]->pos - mesh.edges[e].particles[1]->pos).norm();
        total_constraint += fabs(current_length - mesh.edges[e].rest_length);
    }
    
    return total_constraint;
}

void Triangle::calcArea() {
    Vector3d v1 = particles[1]->pos - particles[0]->pos;
    Vector3d v2 = particles[2]->pos - particles[0]->pos;
    area =  0.5 * (v1.cross(v2)).norm();
}

void Mesh::backupIndex() {
    map<pair<int, int>, int> edge_existence;
    for (int i = 0; i < edges.size(); i++) {
        if (edge_existence.find(make_pair(min(edges[i].particles[0]->index, edges[i].particles[1]->index), max(edges[i].particles[0]->index, edges[i].particles[1]->index))) == edge_existence.end()) {
            edges[i].index[0] = edges[i].particles[0]->index;
            edges[i].index[1] = edges[i].particles[1]->index;
            
            edge_existence[make_pair(min(edges[i].particles[0]->index, edges[i].particles[1]->index), max(edges[i].particles[0]->index, edges[i].particles[1]->index))] = 1;
        } else {
            edges.erase(edges.begin() + i);
            i--;
        }
    }
    cout << "face number: " << faces.size() << endl;
    for (int i = 0; i < faces.size(); i++) {
        faces[i].index[0] = faces[i].particles[0]->index;
        faces[i].index[1] = faces[i].particles[1]->index;
        faces[i].index[2] = faces[i].particles[2]->index;
    }
}

void Mesh::restoreTopology() {
    for (int i = 0; i < edges.size(); i++) {
        edges[i].particles[0] = &particles[edges[i].index[0]];
        edges[i].particles[1] = &particles[edges[i].index[1]];
    }
    
    for (int i = 0; i < faces.size(); i++) {
        faces[i].particles[0] = &particles[faces[i].index[0]];
        faces[i].particles[1] = &particles[faces[i].index[1]];
        faces[i].particles[2] = &particles[faces[i].index[2]];
    }
}



