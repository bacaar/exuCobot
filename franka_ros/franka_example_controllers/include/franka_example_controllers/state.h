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

    const State operator+ (State state2);
    const State operator- (State state2);
    const State operator* (double factor);
    double& operator[] (int i);
    const bool operator==(State state2);
};

// struct representing kinematic state for three dimensions
struct State3{
    State x;
    State y;
    State z;

    const State3 operator+ (State3 state2);
    const State3 operator- (State3 state2);
    const State3 operator* (double factor);
    State& operator[] (int i);
    const bool operator==(State3 state2);
};

#endif //SRC_STATE_H
