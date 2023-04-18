//
// Created by robocup on 17.10.22.
//

#include "franka_example_controllers/samplingPoint.h"

#include <iostream>

/*
 *  Implementations for SamplingPoint-methods
 */

const SamplingPoint SamplingPoint::operator+ (SamplingPoint sp2){
    SamplingPoint res;
    res.pos = pos + sp2.pos;
    res.vel = vel + sp2.vel;
    res.acc = acc + sp2.acc;
    res.jerk = jerk + sp2.jerk;
    return res;
}

const SamplingPoint SamplingPoint::operator- (SamplingPoint sp2){
    SamplingPoint res;
    res.pos = pos - sp2.pos;
    res.vel = vel - sp2.vel;
    res.acc = acc - sp2.acc;
    res.jerk = jerk - sp2.jerk;
    return res;
}

const SamplingPoint SamplingPoint::operator* (double factor){
    SamplingPoint res;
    res.pos = pos * factor;
    res.vel = vel * factor;
    res.acc = acc * factor;
    res.jerk = jerk * factor;
    return res;
}

double& SamplingPoint::operator[] (int i){
    if(i == 0) return pos;
    else if (i == 1) return vel;
    else if (i == 2) return acc;
    else if (i == 3) return jerk;
    else{
        std::cerr << "ERROR: Index " << i << " out of range 4\n";
        exit(-1);
    }
}

const bool SamplingPoint::operator==(SamplingPoint sp2){
    return (pos == sp2.pos && vel == sp2.vel && acc == sp2.acc && jerk == sp2.jerk);
}

/*
 *  Implementations for SamplingPoint3-methods
 */

const SamplingPoint3 SamplingPoint3::operator+ (SamplingPoint3 sp2){
    SamplingPoint3 res;
    res.x = x + sp2.x;
    res.y = y + sp2.y;
    res.z = z + sp2.z;
    return res;
}

const SamplingPoint3 SamplingPoint3::operator- (SamplingPoint3 sp2){
    SamplingPoint3 res;
    res.x = x - sp2.x;
    res.y = y - sp2.y;
    res.z = z - sp2.z;
    return res;
}

const SamplingPoint3 SamplingPoint3::operator* (double factor){
    SamplingPoint3 res;
    res.x = x * factor;
    res.y = y * factor;
    res.z = z * factor;
    return res;
}

SamplingPoint& SamplingPoint3::operator[] (int i){
    if(i == 0) return x;
    else if (i == 1) return y;
    else if (i == 2) return z;
    else{
        std::cerr << "ERROR: Index " << i << " out of range 3\n";
        exit(-1);
    }
}

const bool SamplingPoint3::operator==(SamplingPoint3 sp2){
    return (x == sp2.x && y == sp2.y && z == sp2.z);
}