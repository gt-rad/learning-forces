//
//  MeshReader.h
//  MassSpringLocomotion
//
//  Created by YuWenhao on 4/15/15.
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#ifndef __MassSpringLocomotion__MeshReader__
#define __MassSpringLocomotion__MeshReader__

#include "Mesh.h"
#include <stdio.h>
#include <fstream>
#include <string>
#include <map>

class MeshReader {
public:
    bool readMesh(Mesh& mesh, std::string filename);
    
    bool readObjMesh(Mesh& mesh, std::string filename, Eigen::Vector3d translation = Eigen::Vector3d::Zero(), Eigen::Matrix3d rotate = Eigen::Matrix3d::Identity(), Eigen::Vector3d scale = Eigen::Vector3d::Ones());
    
    bool writeObjMesh(Mesh& mesh, std::string filename);
    
    bool writeRestLength(Mesh& mesh, std::string filename);
    
    bool readRestLength(std::map<std::pair<int, int>, double>&, std::string filename);
};


#endif /* defined(__MassSpringLocomotion__MeshReader__) */
