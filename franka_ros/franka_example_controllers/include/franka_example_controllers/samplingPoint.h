//
// Created by Aaron Bacher on 17.10.22.
//

#ifndef SAMPLINGPOINT_H
#define SAMPLINGPOINT_H

// struct representing samplingPoint for one dimension
struct SamplingPoint{
    double pos = 0.0;
    double vel = 0.0;
    double acc = 0.0;
    double jerk = 0.0;

    const SamplingPoint operator+ (SamplingPoint sp2);   // addition
    const SamplingPoint operator- (SamplingPoint sp2);   // subtraction
    const SamplingPoint operator* (double factor);  // multiplication
    double& operator[] (int i);             // access pos, vel, acc, jerk with indices
    const bool operator==(SamplingPoint sp2);    // comparison, equality check
};

// struct representing kinematic state for three dimensions
struct SamplingPoint3{
    SamplingPoint x;
    SamplingPoint y;
    SamplingPoint z;

    const SamplingPoint3 operator+ (SamplingPoint3 sp2); // addition
    const SamplingPoint3 operator- (SamplingPoint3 sp2); // subtraction
    const SamplingPoint3 operator* (double factor); // multiplication
    SamplingPoint& operator[] (int i);              // access x, y, z with indices
    const bool operator==(SamplingPoint3 sp2);   // comparison, equality check
};

#endif //SAMPLINGPOINT_H
