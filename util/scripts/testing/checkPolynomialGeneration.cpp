/*
Author: Aaron Bacher
Date: 28.09.2022

similar to checkPolynomialGeneration.py but in C++

Script for testing creation of polynomial trajectories
-> reads csv with parameters for trajectory generation (used by c++ controller), recreates them offline again
-> writes result to file
*/

#include <vector>
#include <fstream>
#include <iostream>

#include "csv.h"

// struct representing kinematic state for one dimension
struct State{
    double pos = 0;
    double vel = 0;
    double acc = 0;
    double jerk = 0;
};

// struct representing kinematic state for three dimensions
struct State3{
    State x;
    State y;
    State z;
};

int polynomialDegree_ = 5;

std::vector<double> calcCoefs(State startState, State endState, double T){
    double T2 = T*T;
    double T3 = T2 * T;

    //double s0, double v0, double a0, double sT, double vT, double aT
    std::vector<double> solution(6, 0);

    if(polynomialDegree_ == 3) {
        solution[0] = 0;    // just for easier implementation afterwards
        solution[1] = 0;
        solution[2] = (startState.vel + endState.vel)/T2 + 2*(startState.pos - endState.pos)/T3;
        solution[3] = (-2*startState.vel - endState.vel)/T + 3*(-startState.pos + endState.pos)/T2;
        solution[4] = startState.vel;
        solution[5] = startState.pos;
        //[ds0/T**2 + dsT/T**2 + 2*s0/T**3 - 2*sT/T**3], [-2*ds0/T - dsT/T - 3*s0/T**2 + 3*sT/T**2], [ds0], [s0]]
    }

    if(polynomialDegree_ == 5) {
        double T4 = T3 * T;
        double T5 = T4 * T;

        solution[0] = -startState.acc / (2 * T3) + endState.acc / (2 * T3) - 3 * startState.vel / T4 - 3 * endState.vel / T4 - 6 * startState.pos / T5 + 6 * endState.pos / T5;
        solution[1] = 3 * startState.acc / (2 * T2) - endState.acc / T2 + 8 * startState.vel / T3 + 7 * endState.vel / T3 + 15 * startState.pos / T4 - 15 * endState.pos / T4;
        solution[2] = -3 * startState.acc / (2 * T) + endState.acc / (2 * T) - 6 * startState.vel / T2 - 4 * endState.vel / T2 - 10 * startState.pos / T3 + 10 * endState.pos / T3;
        solution[3] = startState.acc / 2;
        solution[4] = startState.vel;
        solution[5] = startState.pos;
    }

    return solution;
}

/*State evaluatePolynom(std::vector<double> &coef, double t){

    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    State state;

    state.pos  =    coef[0]*t5 +    coef[1]*t4 +   coef[2]*t3 +   coef[3]*t2 + coef[4]*t + coef[5];
    state.vel  =  5*coef[0]*t4 +  4*coef[1]*t3 + 3*coef[2]*t2 + 2*coef[3]*t  + coef[4];
    state.acc  = 20*coef[0]*t3 + 12*coef[1]*t2 + 6*coef[2]*t  + 2*coef[3];
    state.jerk = 60*coef[0]*t2 + 24*coef[1]*t  + 6*coef[2];

    return state;
}*/


int main(){
    io::CSVReader<21> in("/home/aaron/catkin_ws/src/exucobot/log/trajectoryCreation.csv");

    std::ofstream outfile;
    outfile.open("/home/aaron/catkin_ws/src/exucobot/log/coefficientsCpp.csv");
    if(outfile.is_open()){
        outfile << "t,coord,A,B,C,D,E,F,dt\n";
    }
    else{
        std::cerr << "ERROR: Could not open output file!\n";
        exit(-1);
    }

    in.read_header(io::ignore_extra_column,"t","tend","cpx","cvx","cax","cpy","cvy","cay","cpz","cvz","caz","npx","nvx","nax","npy","nvy","nay","npz","nvz","naz","dt");
    double t, tend;
    double cpx,cvx,cax,cpy,cvy,cay,cpz,cvz,caz,npx,nvx,nax,npy,nvy,nay,npz,nvz,naz,dt;
    while(in.read_row(t,tend,cpx,cvx,cax,cpy,cvy,cay,cpz,cvz,caz,npx,nvx,nax,npy,nvy,nay,npz,nvz,naz,dt)){

        std::vector<double> coefsX = calcCoefs({cpx, cvx, cax}, {npx, nvx, nax}, dt);
        std::vector<double> coefsY = calcCoefs({cpy, cvy, cay}, {npy, nvy, nay}, dt);
        std::vector<double> coefsZ = calcCoefs({cpz, cvz, caz}, {npz, nvz, naz}, dt);

        outfile << t << ",x,";
        for(int i = 0; i < 6; ++i){
            outfile << coefsX[i] << ",";
        }
        outfile << dt << std::endl;

        outfile << t << ",y,";
        for(int i = 0; i < 6; ++i){
            outfile << coefsY[i] << ",";
        }
        outfile << dt << std::endl;

        outfile << t << ",z,";
        for(int i = 0; i < 6; ++i){
            outfile << coefsZ[i] << ",";
        }
        outfile << dt << std::endl;
    }
}