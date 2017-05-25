/*
  Copyright ©2013 The Regents of the University of California
  (Regents). All Rights Reserved. Permission to use, copy, modify, and
  distribute this software and its documentation for educational,
  research, and not-for-profit purposes, without fee and without a
  signed licensing agreement, is hereby granted, provided that the
  above copyright notice, this paragraph and the following two
  paragraphs appear in all copies, modifications, and
  distributions. Contact The Office of Technology Licensing, UC
  Berkeley, 2150 Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620,
  (510) 643-7201, for commercial licensing opportunities.

  IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT,
  INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING
  LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
  DOCUMENTATION, EVEN IF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY
  OF SUCH DAMAGE.

  REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE. THE SOFTWARE AND ACCOMPANYING
  DOCUMENTATION, IF ANY, PROVIDED HEREUNDER IS PROVIDED "AS
  IS". REGENTS HAS NO OBLIGATION TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
*/

#include <iostream>
#include "spline.hpp"
#include "stdafx.h"
#include <PxPhysicsAPI.h>

// binary search, returns keyframe immediately *after* given time
// range of output: 0 to a.keyfs.size() inclusive
template<typename T>
static int find (const Spline<T> &s, double t) {
    int l = 0, u = s.points.size();
    while (l != u) {
         int m = (l + u)/2;
         if (t < s.points[m].t) u = m;
         else l = m + 1;
    }
    return l; // which is equal to u
}

// Returns the velocity at index i
template<typename T>
T Spline<T>::get_velocity(int i) const {
    if (i == 0){
		if(points.size() < 2)
			return T(0);
		return (points[i+1].x - points[i].x) / (points[i+1].t - points[i].t); //linear
    } else if(i+1 >= points.size()) {
		return (points[i].x - points[i-1].x) / (points[i].t - points[i-1].t); //linear
	}else {
         return (points[i+1].x - points[i-1].x) / (points[i+1].t - points[i-1].t); //curve tangent
    }
}

template<>
physx::PxQuat Spline<physx::PxQuat>::get_velocity(int i) const{
    if (i == 0){
		if(points.size() < 2)
			return physx::PxQuat(1,0,0,0);
		return (points[i+1].x - points[i].x) *(1.0/ (points[i+1].t - points[i].t)); //linear
    } else if(i+1 >= points.size()) {
		return (points[i].x - points[i-1].x) *(1.0/  (points[i].t - points[i-1].t)); //linear
	}else {
         return (points[i+1].x - points[i-1].x) *(1.0/ (points[i+1].t - points[i-1].t)); //curve tangent
    }
}

template<>
Eigen::VectorXd Spline<Eigen::VectorXd>::get_velocity(int i) const{
    if (i == 0){
		if(points.size() < 2)
			return Eigen::VectorXd::Zero(points[0].x.size());
		return (points[i+1].x - points[i].x) / (points[i+1].t - points[i].t); //linear
    } else if(i+1 >= points.size()) {
		return (points[i].x - points[i-1].x) / (points[i].t - points[i-1].t); //linear
	}else {
         return (points[i+1].x - points[i-1].x) / (points[i+1].t - points[i-1].t); //curve tangent
    }
}

template<>
Eigen::Vector3d Spline<Eigen::Vector3d>::get_velocity(int i) const{
   if (i == 0){
		if(points.size() < 2)
			return Eigen::Vector3d::Zero();
		return (points[i+1].x - points[i].x) / (points[i+1].t - points[i].t); //linear
    } else if(i+1 >= points.size()) {
		return (points[i].x - points[i-1].x) / (points[i].t - points[i-1].t); //linear
	}else {
         return (points[i+1].x - points[i-1].x) / (points[i+1].t - points[i-1].t); //curve tangent
    }
}

template<typename T>
T Spline<T>::pos (double t) const {
    int i = find(*this, t);
    if (i == 0) {
		const Point &p1 = points.front();
         return p1.x;
    } else if (i == points.size()) {
		const Point &p0 = points.back();
         return p0.x;
    } else {
         const Point &p0 = points[i-1], &p1 = points[i];
         T p0_v = get_velocity(i-1);
         T p1_v = get_velocity(i);
         double s = (t - p0.t)/(p1.t - p0.t), s2 = s*s, s3 = s2*s;
         return p0.x*(2*s3 - 3*s2 + 1) + p1.x*(-2*s3 + 3*s2)
                  + (p0_v*(s3 - 2*s2 + s) + p1_v*(s3 - s2))*(p1.t - p0.t);
    }
}

template <typename T>
T Spline<T>::vel (double t) const {
    int i = find(*this, t);
    if (i == 0 || i == points.size()) {
        return get_velocity(i);
    } else {
        const Point &p0 = points[i-1], &p1 = points[i];
        T p0_v = get_velocity(i-1);
        T p1_v = get_velocity(i);
        double s = (t - p0.t)/(p1.t - p0.t), s2 = s*s;
        return (p0.x*(6*s2 - 6*s) + p1.x*(-6*s2 + 6*s))/(p1.t - p0.t)
            + p0_v*(3*s2 - 4*s + 1) + p1_v*(3*s2 - 2*s);
    }
}

template <>
physx::PxQuat Spline<physx::PxQuat>::vel (double t) const {
    int i = find(*this, t);
    if (i == 0 || i == points.size()) {
        return get_velocity(i);
    } else {
        const Point &p0 = points[i-1], &p1 = points[i];
        physx::PxQuat p0_v = get_velocity(i-1);
        physx::PxQuat p1_v = get_velocity(i);
        double s = (t - p0.t)/(p1.t - p0.t), s2 = s*s;
        return (p0.x*(6*s2 - 6*s) + p1.x*(-6*s2 + 6*s))* (1.0/(p1.t - p0.t))
            + p0_v*(3*s2 - 4*s + 1) + p1_v*(3*s2 - 2*s);
    }
}

template <typename T>
void Spline<T>::addPoint(double t){
    std::vector<Spline<T>::Point> newPoints;
    bool found = false;
    for(int i=0; i<points.size(); i++){
        if(points[i].t > t && !found){
            //insert the new point
            Spline<T>::Point p;
            p.t = t;
            p.x = pos(t);
            newPoints.push_back(p);
            found = true;
        }
        newPoints.push_back(points[i]);
    }
    points = newPoints;
}

template <typename T>
void Spline<T>::insert(double _t, T _x){
    std::vector<Spline<T>::Point> newPoints;
	bool found = false;
	Spline<T>::Point p;
    p.t = _t;
    p.x = _x;
    for(int i=0; i<points.size(); i++){
        if(points[i].t > _t && !found){
            //insert the new point
            newPoints.push_back(p);
            found = true;
        }
        newPoints.push_back(points[i]);
    }
	if(!found)
		newPoints.push_back(p);
    points = newPoints;
}

template class Spline<Eigen::VectorXd>;
template class Spline<Eigen::Vector3d>;
template class Spline<physx::PxQuat>;
