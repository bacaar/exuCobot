//
// Created by robocup on 17.10.22.
//

#include "franka_example_controllers/state.h"

#include <iostream>

/*
 *  Implementations for Methods of State
 */

const State State::operator+ (State state2){
    State res;
    res.pos = pos + state2.pos;
    res.vel = vel + state2.vel;
    res.acc = acc + state2.acc;
    res.jerk = jerk + state2.jerk;
    return res;
}

const State State::operator* (double factor){
    State res;
    res.pos = pos * factor;
    res.vel = vel * factor;
    res.acc = acc * factor;
    res.jerk = jerk * factor;
    return res;
}

const State State::operator- (State state2){
    State res;
    res.pos = pos - state2.pos;
    res.vel = vel - state2.vel;
    res.acc = acc - state2.acc;
    res.jerk = jerk - state2.jerk;
    return res;
}

double& State::operator[] (int i){
    if(i == 0) return pos;
    else if (i == 1) return vel;
    else if (i == 2) return acc;
    else if (i == 3) return jerk;
    else{
        std::cerr << "ERROR: Index " << i << " out of range 4\n";
        exit(-1);
    }
}

const bool State::operator==(State state2){
    return (pos == state2.pos && vel == state2.vel && acc == state2.acc && jerk == state2.jerk);
}

/*
 *  Implementations for Methods of State3
 */

const State3 State3::operator+ (State3 state2){
    State3 res;
    res.x = x + state2.x;
    res.y = y + state2.y;
    res.z = z + state2.z;
    return res;
}

const State3 State3::operator* (double factor){
    State3 res;
    res.x = x * factor;
    res.y = y * factor;
    res.z = z * factor;
    return res;
}

const State3 State3::operator- (State3 state2){
    State3 res;
    res.x = x - state2.x;
    res.y = y - state2.y;
    res.z = z - state2.z;
    return res;
}

State& State3::operator[] (int i){
    if(i == 0) return x;
    else if (i == 1) return y;
    else if (i == 2) return z;
    else{
        std::cerr << "ERROR: Index " << i << " out of range 3\n";
        exit(-1);
    }
}

const bool State3::operator==(State3 state2){
    return (x == state2.x && y == state2.y && z == state2.z);
}