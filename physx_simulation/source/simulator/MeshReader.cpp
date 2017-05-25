//
//  MeshReader.cpp
//  MassSpringLocomotion
//
//  Created by YuWenhao on 4/15/15.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#include "MeshReader.h"
#include <map>
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;
using namespace Eigen;

bool MeshReader::readMesh(Mesh& mesh, string filename) {
    ifstream ifile(filename.c_str());
    if (!ifile.good()) {
        cout << "Open file: " << filename << " failed\n";
        abort();
    }
    
    mesh.cleanMesh();
    
    vector<Particle> & node_list = mesh.particles;
    vector<Edge> & edge_list = mesh.edges;
    vector<Triangle> & face_list = mesh.faces;
    
    string type;
    int node_num, edge_num, face_num;
    double x, y, z;
    int ind1, ind2, ind3;
    
    // read nodes
    ifile >> type;
    if (type.compare("Node:") != 0) {
        cout << "missing node type\n";
        abort();
    }
    ifile >> node_num;
    for (int i = 0; i < node_num; i++) {
        Particle node;
        node.index = i;
        ifile >> x >> y >> z;
        node.pos = 10*Vector3d(x, y, z);
        node_list.push_back(node);
    }
    
    // read edges
    ifile >> type;
    if (type.compare("Edge:") != 0) {
        cout << "missing edge type\n";
        abort();
    }
    ifile >> edge_num;
    for (int i = 0; i < edge_num; i++) {
        Edge edge;
        ifile >> ind1 >> ind2;
        edge.particles[0] = &node_list[ind1];
        edge.particles[1] = &node_list[ind2];
        edge.rest_length = (edge.particles[0]->pos - edge.particles[1]->pos).norm();
        edge_list.push_back(edge);
    }
    
    // read faces
    ifile >> type;
    if (type.compare("Face:") != 0) {
        cout << "missing face type\n";
        abort();
    }
    ifile >> face_num;
    for (int i = 0; i < face_num; i++) {
        Triangle face;
        ifile >> ind1 >> ind2 >> ind3;
        face.particles[0] = &node_list[ind1];
        face.particles[1] = &node_list[ind2];
        face.particles[2] = &node_list[ind3];
        
        face_list.push_back(face);
    }
    
    ifile.close();
    
    // re-initialize the edges
    edge_list.clear();
    vector<map<int, int> > edge_map;
    for (int i = 0; i < node_list.size(); i++) {
        edge_map.push_back(map<int, int>());
    }
    for (int i = 0; i < face_list.size(); i++) {
        int index[3];
        index[0] = face_list[i].particles[0]->index;
        index[1] = face_list[i].particles[1]->index;
        index[2] = face_list[i].particles[2]->index;
        for (int j = 0; j < 3; j++) {
            if (edge_map[index[j]][index[(j+1)%3]] != 0) {
                continue;
            }
            Edge edge;
            edge.particles[0] = &node_list[index[j]];
            edge.particles[1] = &node_list[index[(j+1)%3]];
            edge.rest_length = 1 *(edge.particles[0]->pos - edge.particles[1]->pos).norm();
            
            edge_map[index[j]][index[(j+1)%3]] = 10;
            edge_map[index[(j+1)%3]][index[j]] = 10;
            edge_list.push_back(edge);
        }
    }
    
    for (int i = 0; i < node_list.size(); i++) {
        node_list[i].pos(0) *= -0.7;
        node_list[i].pos(1) *= 1.3;
    }
    
    return true;
}

bool MeshReader::readObjMesh(Mesh& mesh, string filename, Vector3d translate, Matrix3d rotate, Vector3d scale) {
    ifstream ifile(filename.c_str());
    if (!ifile.good()) {
        cout << "Open file: " << filename << " failed\n";
        abort();
    }
    
    string type;
    string val;
    
    mesh.cleanMesh();
    int particle_index = 0;
    while (!ifile.eof()) {
        ifile >> type;
        if (ifile.eof()) break;
        if (type == "v") {
            double x, y, z;
            ifile >> x >> y >> z;
            Particle part;
            part.index = particle_index++;
            part.pos = Vector3d(x, y, z);
            
            for (int j = 0; j < 3; j++) {
                part.pos(j) *= scale(j);
            }
            part.pos = rotate * part.pos;
            
            for (int j = 0; j < 3; j++) {
                part.pos(j) += translate(j);
            }
            
            mesh.particles.push_back(part);
        } else if (type == "f") {
            ifile >> val;
            stringstream st1(val);
            int index1, index2, index3;
            st1 >> index1;
            
            ifile >> val;
            stringstream st2(val);
            st2 >> index2;
            
            ifile >> val;
            stringstream st3(val);
            st3 >> index3;
            
            mesh.faces.push_back(Triangle(&mesh.particles[index1-1], &mesh.particles[index3-1], &mesh.particles[index2-1]));
            mesh.faces[mesh.faces.size()-1].index[0] = index1 - 1;
            mesh.faces[mesh.faces.size()-1].index[1] = index3 - 1;
            mesh.faces[mesh.faces.size()-1].index[2] = index2 - 1;
        }
    }
    
    mesh.restoreTopology();
    
    ifile.close();
    
    return true;
}

bool MeshReader::writeObjMesh(Mesh& mesh, string filename) {
    ofstream ofile(filename.c_str());
    if (!ofile.good()) {
        cout << "Open file: " << filename << " failed\n";
        abort();
    }
    
    for (int i = 0; i < mesh.particles.size(); i++) {
        ofile << "v " << mesh.particles[i].pos(0) << " " << mesh.particles[i].pos(1) << " " << mesh.particles[i].pos(2) << endl;
    }
    
    for (int i = 0; i < mesh.particles.size(); i++) {
        ofile << "vn " << mesh.particles[i].normal(0) << " " << mesh.particles[i].normal(1) << " " << mesh.particles[i].normal(2) << endl;
    }
    
    for (int i = 0; i < mesh.faces.size(); i++) {
        ofile << "f " << mesh.faces[i].particles[0]->index+1 << "/" << mesh.faces[i].particles[0]->index+1 << " " << mesh.faces[i].particles[2]->index+1 << "/" << mesh.faces[i].particles[2]->index+1 << " " << mesh.faces[i].particles[1]->index+1 << "/" << mesh.faces[i].particles[1]->index+1 << endl;
    }
    
    ofile.close();
    
    return true;
}

bool MeshReader::writeRestLength(Mesh& mesh, string filename) {
    ofstream ofile(filename);

    for (int i = 0; i < mesh.edges.size(); i++) {
        ofile << mesh.edges[i].particles[0]->index << " " << mesh.edges[i].particles[1]->index << " " << mesh.edges[i].rest_length << endl;
    }
    
    ofile.close();
    
    return true;
}

bool MeshReader::readRestLength(map<pair<int, int>, double>& edge_length_map, string filename) {
    edge_length_map.clear();
    
    ifstream ifile(filename);
    
    if (!ifile.good()) {
        return false;
    }
    
    while (!ifile.eof()) {
        int ind1, ind2;
        double val;
        ifile >> ind1 >> ind2 >> val;
        edge_length_map[make_pair(min(ind1, ind2), max(ind1, ind2))] = val;
    }
    
    ifile.close();
    return true;
}






