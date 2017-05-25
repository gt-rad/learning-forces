//
//  Renderer.cpp
//  physx_test
//
//  Created by YuWenhao on 10/1/15.
//  Adapted by Zackory Erickson
//  Copyright (c) 2015 YuWenhao. All rights reserved.
//

#include "Renderer.h"
#include "../utils/opengl.h"
#include "../simulator/PxSphereManager.h"
#include <iostream>

using namespace std;

void Renderer::renderMesh(Mesh& mesh, vector<float> color, bool wire, bool seqinv) {
    // glTexEnvf(GL_TEXTURE_2D,GL_TEXTURE_ENV_MODE,GL_MODULATE);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glColorMask(true, true, true, false);
    glColor4f(color[0], color[1], color[2], 0.5);
    // glColor4f(color[0], color[1], color[2], 1.0);
    
    mesh.updateNorm();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < mesh.faces.size(); i++) {
        if (seqinv) {
            for (int j = 0; j < 3; j++) {
                glNormal3d(mesh.faces[i].particles[j]->normal(0), mesh.faces[i].particles[j]->normal(1), mesh.faces[i].particles[j]->normal(2));
                glVertex3d(mesh.faces[i].particles[j]->pos(0), mesh.faces[i].particles[j]->pos(1), mesh.faces[i].particles[j]->pos(2));
            }
        } else {
            for (int j = 2; j >= 0; j--) {
                glNormal3d(mesh.faces[i].particles[j]->normal(0), mesh.faces[i].particles[j]->normal(1), mesh.faces[i].particles[j]->normal(2));
                glVertex3d(mesh.faces[i].particles[j]->pos(0), mesh.faces[i].particles[j]->pos(1), mesh.faces[i].particles[j]->pos(2));
            }
        }
    }
    glEnd();
    glDepthMask(GL_TRUE);
    
    if (wire) {
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        glColor4d(0.8, 0.8, 0.7, 0.5);
        // glColor4f(color[0], color[1], color[2], 0.5);
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < mesh.faces.size(); i++) {
            if (seqinv) {
                for (int j = 0; j < 3; j++) {
                    glNormal3d(mesh.faces[i].particles[j]->normal(0), mesh.faces[i].particles[j]->normal(1), mesh.faces[i].particles[j]->normal(2));
                    glVertex3d(mesh.faces[i].particles[j]->pos(0), mesh.faces[i].particles[j]->pos(1), mesh.faces[i].particles[j]->pos(2));
                }
            } else {
                for (int j = 2; j >= 0; j--) {
                    glNormal3d(mesh.faces[i].particles[j]->normal(0), mesh.faces[i].particles[j]->normal(1), mesh.faces[i].particles[j]->normal(2));
                    glVertex3d(mesh.faces[i].particles[j]->pos(0), mesh.faces[i].particles[j]->pos(1), mesh.faces[i].particles[j]->pos(2));
                }
            }
        }
        glEnd();
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
    }
    glDisable(GL_BLEND);
}

void Renderer::renderRigPart(RigPart* part, vector<float> color) {
    for (int i = 0; i < part->descriptors.size(); i++) {
        renderShape(part->descriptors[i], color);
    }
    
    // draw force torque sensor
    /*glColor3d(1, 0, 0);
    float scale = 0.1;
    glBegin(GL_LINES);
    glVertex3d(part->ft_center[0], part->ft_center[1], part->ft_center[2]);
    glVertex3d(part->ft_center[0]+part->total_force[0]*scale, part->ft_center[1]+part->total_force[1]*scale, part->ft_center[2]+part->total_force[2]*scale);
    glEnd();
    
    glColor3d(0, 1, 0);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex3d(part->ft_center[0], part->ft_center[1], part->ft_center[2]);
    glVertex3d(part->ft_center[0]+part->total_torque[0], part->ft_center[1]+part->total_torque[1], part->ft_center[2]+part->total_torque[2]);
    glEnd();
    glLineWidth(1);*/
    
    // draw gripped points
    /*glColor3d(0, 0, 1);
    for (int i = 0; i < part->gripper_constraints.size(); i++) {
        glPushMatrix();
        glTranslated(part->gripper_constraints[i].second[0], part->gripper_constraints[i].second[1], part->gripper_constraints[i].second[2]);
        glutSolidSphere(0.005, 10, 10);
        glPopMatrix();
    }*/
}

void Renderer::renderRigMotion(RigPart* part, std::vector<float> color){
	glColor4d(1, 1, 1, 0.5);
	//std::cout << "render rig motion" << std::endl;
	if(!part->is_motion_rig)
		return;
	//std::cout << "render rig motion" << std::endl;
	for(double d=part->translation_spline.points[0].t; d<part->translation_spline.points.back().t; d+=0.05){
		glPushMatrix();
		Eigen::Vector3d pos = part->translation_spline.pos(d);
        glTranslatef(pos(0), pos(1), pos(2));
        glutSolidSphere(0.01, 30, 30);
        glPopMatrix();
	}

	glColor4d(0.55, 0.75, 0.3, 0.5);
	for(int i=1; i<part->translation_spline.points.size(); i++){
		Eigen::Vector3d pos = part->translation_spline.points[i].x;
		glPushMatrix();
		glTranslatef(pos(0), pos(1), pos(2));
        glutSolidSphere(0.015, 30, 30);
        glPopMatrix();

	}
}

void Renderer::renderShape(RenderDescriptor* desc, vector<float> color) {
    glColor3d(color[0], color[1], color[2]);
    
    glPushMatrix();
    glTranslated(desc->translation[0], desc->translation[1], desc->translation[2]);
    physx::PxReal angle;
    physx::PxVec3 axis;
    desc->rotation.toRadiansAndUnitAxis(angle, axis);
    glRotated(angle * 180 / physx::PxPi, axis[0], axis[1], axis[2]);
    glScaled(desc->scale[0], desc->scale[1], desc->scale[2]);
    
    if (desc->type == RenderDescriptor::rCapsule) {
        double hheight = ((CapsuleDescriptor*)desc)->half_height,
        radius = ((CapsuleDescriptor*)desc)->radius;
        int divide = 200;
        int hdivide = divide;
        glTranslated(hheight, 0, 0);
        glutSolidSphere(radius, divide/2, divide/2);
        glTranslated(-2*hheight, 0, 0);
        glutSolidSphere(radius, divide/2, divide/2);
        
        glBegin(GL_QUADS);
        for (int i = 0; i < divide; i++) {
            double y1 = radius * sin(physx::PxPi*2.0/divide * i);
            double z1 = radius * cos(physx::PxPi*2.0/divide * i);
            double y2 = radius * sin(physx::PxPi*2.0/divide * ((i+1)%divide));
            double z2 = radius * cos(physx::PxPi*2.0/divide * ((i+1)%divide));
            physx::PxVec3 norm(0, y1+y2, z1+z2);
            norm.normalize();
            for (int j = 0; j < hdivide; j++) {
                glNormal3d(norm[0], norm[1], norm[2]);
                glVertex3d(j * hheight*2 / hdivide, y1, z1);
                glNormal3d(norm[0], norm[1], norm[2]);
                glVertex3d(j * hheight*2 / hdivide, y2, z2);
                glNormal3d(norm[0], norm[1], norm[2]);
                glVertex3d((j+1) * hheight*2 / hdivide, y2, z2);
                glNormal3d(norm[0], norm[1], norm[2]);
                glVertex3d((j+1) * hheight*2 / hdivide, y1, z1);
            }
        }
        glEnd();
        
    } else if (desc->type == RenderDescriptor::rBox) {
        glScaled(0.99, 0.99, 0.99);
        glScaled(((BoxDescriptor*)desc)->half_width*2, ((BoxDescriptor*)desc)->half_height*2, ((BoxDescriptor*)desc)->half_length*2);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glColor4f(0.55, 0.75, 0.3, 0.15);
        glutSolidCube(1);
        glColor4f(0.55, 0.75, 0.3, 1);
        glutWireCube(1);
        glDisable(GL_BLEND);
    }
    
    glPopMatrix();
}

void Renderer::renderForceMap(float alpha) {
    // Render each force "pixel" on the screen to create a heatmap of forces
    vector<physx::PxVec4> forcemap = PxSphereManager::GetSingleton()->getForceMap();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(1.0, 0.0, 0.0, alpha);
    for (int i = 0; i < forcemap.size(); i++) {
        physx::PxVec3 pos = forcemap[i].getXYZ();
        float magnitude = forcemap[i].w;
        glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        // Use sphere radius of 10 times the magnitude of force
        glutSolidSphere(magnitude, 30, 30);
        glPopMatrix();
    }
    glDisable(GL_BLEND);
}

void Renderer::renderSphereManager(vector<float> color, bool drawSpheres) {
    vector<physx::PxClothCollisionSphere> spheres;
    vector<pair<uint32_t, uint32_t> > capsules;
    
    PxSphereManager::GetSingleton()->getSphereData(spheres, capsules);
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4d(color[0], color[1], color[2], color[3]);
    if (drawSpheres) {
        for (int i = 0; i < spheres.size(); i++) {
            glPushMatrix();
            glTranslated(spheres[i].pos.x, spheres[i].pos.y, spheres[i].pos.z);
            glutSolidSphere(spheres[i].radius - 0.001, 20, 20);
            glPopMatrix();
        }
    }
    
    for (int i = 0; i < capsules.size(); i++) {
        physx::PxVec3 cent1 = spheres[capsules[i].first].pos;
        physx::PxVec3 cent2 = spheres[capsules[i].second].pos;
        double radius1 = spheres[capsules[i].first].radius + (drawSpheres ? -0.001 : 0);
        double radius2 = spheres[capsules[i].second].radius + (drawSpheres ? -0.001 : 0);
        
        physx::PxVec3 direction = cent2 - cent1;
        double cent_distance = direction.magnitude();
        
        //x,y,z   y,-x,0
        physx::PxVec3 unit_direction = direction.getNormalized();
        physx::PxVec3 normal_base1;
        if (unit_direction.x != 0 || unit_direction.y != 0) {
            normal_base1 = physx::PxVec3(unit_direction.y, -unit_direction.x, 0).getNormalized();
        } else { // is (0, 0, 1)
            normal_base1 = physx::PxVec3(1, 0, 0);
        }
        physx::PxVec3 normal_base2 = unit_direction.cross(normal_base1).getNormalized();
        
        double d1, d2;
        d1 = radius1*(radius1-radius2) / cent_distance;
        d2 = radius2*(radius1-radius2) / cent_distance;
        
        
        double draw_radiu1 = sqrt(radius1*radius1 - d1*d1);
        double draw_radiu2 = sqrt(radius2*radius2 - d2*d2);
        physx::PxVec3 draw_cent1 = cent1 + d1 * unit_direction;
        physx::PxVec3 draw_cent2 = cent2 + d2 * unit_direction;
        
        int subdivision = 200;
        int hdivide = 2;
        for (int j = 0; j < subdivision; j++) {
            double x1 = sin(6.28/subdivision*1.0*j), y1 = cos(6.28/subdivision*1.0*j),
                    x2 = sin(6.28/subdivision*1.0*(j+1)), y2 = cos(6.28/subdivision*1.0*(j+1));
            
            physx::PxVec3 p1 = draw_cent1 + x1 * draw_radiu1 * normal_base1 + draw_radiu1 * y1 * normal_base2;
            physx::PxVec3 p2 = draw_cent1 + x2 * draw_radiu1 * normal_base1 + draw_radiu1 * y2 * normal_base2;
            physx::PxVec3 p3 = draw_cent2 + x1 * draw_radiu2 * normal_base1 + draw_radiu2 * y1 * normal_base2;
            physx::PxVec3 p4 = draw_cent2 + x2 * draw_radiu2 * normal_base1 + draw_radiu2 * y2 * normal_base2;
            
            physx::PxVec3 normal = (p1+p2-draw_cent1*2).getNormalized();
            
            
            for (int k = 0; k < hdivide; k++) {
                physx::PxVec3 pp1 = (p3-p1)/hdivide*k+p1;
                physx::PxVec3 pp2 = (p4-p2)/hdivide*k+p2;
                physx::PxVec3 pp3 = (p3-p1)/hdivide*(k+1)+p1;
                physx::PxVec3 pp4 = (p4-p2)/hdivide*(k+1)+p2;
                glBegin(GL_QUADS);
                glNormal3d(normal.x, normal.y, normal.z);
                glVertex3d(pp1.x, pp1.y, pp1.z);
                glNormal3d(normal.x, normal.y, normal.z);
                glVertex3d(pp2.x, pp2.y, pp2.z);
                glNormal3d(normal.x, normal.y, normal.z);
                glVertex3d(pp4.x, pp4.y, pp4.z);
                glNormal3d(normal.x, normal.y, normal.z);
                glVertex3d(pp3.x, pp3.y, pp3.z);
                glEnd();
            }
            
        }
    }
    glDisable(GL_BLEND);
}



