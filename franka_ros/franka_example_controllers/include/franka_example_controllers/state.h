//
// Created by Aaron Bacher on 17.10.22.
//

#ifndef SRC_STATE_H
#define SRC_STATE_H

// struct representing kinematic state for one dimension
struct State{
    double pos = 0.0;
    double vel = 0.0;
    double acc = 0.0;
    double jerk = 0.0;

    const State operator+ (State state2);   // addition
    const State operator- (State state2);   // subtraction
    const State operator* (double factor);  // multiplication
    double& operator[] (int i);             // access pos, vel, acc, jerk with indices
    const bool operator==(State state2);    // comparison, equality check
};

// struct representing kinematic state for three dimensions
struct State3{
    State x;
    State y;
    State z;

    const State3 operator+ (State3 state2); // addition
    const State3 operator- (State3 state2); // subtraction
    const State3 operator* (double factor); // multiplication
    State& operator[] (int i);              // access x, y, z with indices
    const bool operator==(State3 state2);   // comparison, equality check
};

#endif //SRC_STATE_H
