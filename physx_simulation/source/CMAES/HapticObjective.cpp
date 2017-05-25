//
//  HapticObjective.cpp
//  physx_test
//
//  Created by YuWenhao on 1/9/16.
//  Copyright (c) 2016 YuWenhao. All rights reserved.
//

#include "HapticObjective.h"
#include <fstream>
#include <iostream>

using namespace std;

double HapticObjective::evalObjective(baseSimulator* sim) {
    double score = 0;
    gownSimulator* nsim = (gownSimulator*)sim;
    if (nsim->recorded_positiosn.size() == 0) {
        nsim->recorded_positiosn.push_back(0);
        nsim->recorded_positiosn.push_back(1);
        
        nsim->rig_parts[1].recorded_forces.push_back(physx::PxVec3(0));
        nsim->rig_parts[1].recorded_forces.push_back(physx::PxVec3(0));
    }
    double orig = nsim->recorded_positiosn[0];
    for (int i = 0; i < nsim->recorded_positiosn.size(); i++) {
        nsim->recorded_positiosn[i] = fabs(nsim->recorded_positiosn[i] - orig);
    }
    
    vector<double> sim_fmove, sim_fgravity;
    for (int i = 0; i < nsim->recorded_positiosn.size(); i++) {
        sim_fmove.push_back(nsim->rig_parts[1].recorded_forces[i].x);
        sim_fgravity.push_back(nsim->rig_parts[1].recorded_forces[i].y);
    }
    return evalObj(nsim->recorded_positiosn, sim_fmove, sim_fgravity);
}

void HapticObjective::Clear() {
    exp_time.clear();
    exp_position.clear();
    exp_fmove.clear();
    exp_flateral.clear();
    exp_fgravity.clear();
}

void HapticObjective::ReadData(string filename) {
    Clear();
    ifstream ifile(filename.c_str());
    if (!ifile.good()) {
        cout << "Data file: " << filename << " doesn't exist!" << endl;
        return;
    }
    
    while (!ifile.eof()) {
        double time, pos, fmove, flateral, fgravity;
        ifile >> time >> pos >> fmove >> flateral >> fgravity;
        
        exp_time.push_back(time);
        exp_position.push_back(pos);
        exp_fmove.push_back(fmove);
        exp_flateral.push_back(flateral);
        exp_fgravity.push_back(fgravity);
    }
    cout << exp_fgravity.size() << endl;
    ifile.close();
}

void HapticObjective::getInterpolationFrac(const std::vector<double>& ref_vec, double val, int& ind1, double& frac1, int& ind2, double& frac2) {
    int fst = 0, snd = ref_vec.size()-1;
    while (snd - fst > 1) {
        if (val < ref_vec[(fst+snd)/2]) {
            snd = (fst+snd)/2;
        } else if (val > ref_vec[(fst+snd)/2]){
            fst = (fst+snd)/2;
        } else {
            fst = snd = (fst+snd)/2;
        }
    }
    
    ind1 = fst;
    ind2 = snd;
    if (val < ref_vec[fst]) {
        frac1 = 1;
        frac2 = 0;
    } else if (val > ref_vec[snd]) {
        frac1 = 0;
        frac2 = 1;
    } else {
        if (fabs((ref_vec[snd] - ref_vec[fst])) < 0.000001) {
            //cout << "close: " << ref_vec[snd] << " " << ref_vec[fst] << " " << fst << " " << snd << endl;
            frac1 = frac2 = 0.5;
        } else {
            frac2 = (val - ref_vec[fst]) / (ref_vec[snd] - ref_vec[fst]);
            frac1 = 1 - frac2;
        }
    }
}

double HapticObjective::evalObj(const vector<double>& sim_position, const vector<double>& sim_fmove, const vector<double>& sim_fgravity) {
    double score = 0;
    for (int i = 1; i < exp_position.size(); i++) {
        int ind1, ind2;
        double frac1, frac2;
        getInterpolationFrac(sim_position, exp_position[i], ind1, frac1, ind2, frac2);
        double interp_move, interp_gravity;
        interp_move = sim_fmove[ind1] * frac1 + sim_fmove[ind2] * frac2;
        interp_gravity = sim_fgravity[ind1] * frac1 + sim_fgravity[ind2] * frac2;
        
        if (exp_position[i] > sim_position[sim_position.size()-1]) {
            double weight = exp_position[i] + sim_position[sim_position.size()-1];
            weight = exp(weight);
            double diff = fabs(exp_fmove[i] + sim_fmove[sim_position.size()-1]) + fabs(exp_fgravity[i] - sim_fgravity[sim_position.size()-1]);
            score += diff*1.5 * weight * (exp_position[i] - exp_position[i-1]);
        } else {
            double weight = exp(exp_position[i]);
            //weight = 1;
            
            // use higher weights
            if (exp_position[i] >= 0.4 && exp_position[i] <= 0.8) {
                //weight *= 20;
            }
            
            score += weight * fabs(exp_fmove[i] + interp_move) * (exp_position[i]-exp_position[i-1]);
            score += weight * fabs(exp_fgravity[i] - interp_gravity) * (exp_position[i]-exp_position[i-1]);
        }
    }
    
    if (exp_position[exp_position.size()-1] < sim_position[sim_position.size()-1]) {
        double exp_lastmove = exp_fmove[exp_fmove.size()-1];
        double exp_lastgrav = exp_fgravity[exp_fgravity.size()-1];
        
        int ind1, ind2;
        double frac1, frac2;
        getInterpolationFrac(sim_position, exp_position[exp_position.size()-1], ind1, frac1, ind2, frac2);
        
        for (int i = ind2; i < sim_position.size(); i++) {
            double weight = sim_position[i] - exp_position[exp_position.size()-1];
            weight = exp(weight);
            double diff = fabs(exp_fmove[exp_position.size()-1] + sim_fmove[i]) + fabs(exp_fgravity[exp_position.size()-1] - sim_fgravity[i]);
            
            score += diff*1.5 * weight * (sim_position[i] - sim_position[i-1]);
        }
    }
    
    return score;
}

double HapticObjective::sanityCheck() {
    for (int i = 0; i < exp_position.size(); i++) {
        exp_fmove[i] = 0;
        exp_fgravity[i] = 1;
    }
    vector<double> new_exp_fmove = exp_fmove;
    vector<double> new_exp_fgravity = exp_fmove;
    vector<double> new_pos = exp_position;
    
    cout << evalObj(new_pos, new_exp_fmove, exp_fgravity) << " should be 0" << endl;
    
    for (int i = 0; i < exp_position.size(); i++) {
        new_exp_fmove[i] += 0.001;
    }
    cout << evalObj(new_pos, new_exp_fmove, exp_fgravity) << " when fmove = 0.001" << endl;
    
    for (int i = 0; i < exp_position.size(); i++) {
        new_pos[i] += 0.1;
    }
    cout << evalObj(new_pos, new_exp_fmove, exp_fgravity) << " when fmove = 0.001 and move 0.1+" << endl;
    
    return evalObj(new_pos, new_exp_fmove, exp_fgravity);
}





