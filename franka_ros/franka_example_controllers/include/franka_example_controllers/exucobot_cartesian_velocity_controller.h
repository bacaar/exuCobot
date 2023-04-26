// Aaron Bacher, March 2022 - March 2023
// aaronbacher@gmx.de
//
// based on example code from Franka Emika
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#pragma once

#include <array>
#include <memory>
#include <string>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <util/posVelAccJerk3dStamped.h>
#include <util/segmentCommand.h>

#include "franka_example_controllers/samplingPoint.h"

#include "franka_example_controllers/Logger.h"

#define ENABLE_LOGGING 0

// struct representing a command received by Exudyn
struct Command{
    SamplingPoint3 sp;   // sampling point holding pos, vel and acc for all three axes
    double dt;      // time in which new sampling point should be reached

    // operator to be able to iterate over three axes
    SamplingPoint operator[] (int i){
        if(i == 0) return sp.x;
        else if (i == 1) return sp.y;
        else if (i == 2) return sp.z;
        else{
            std::cerr << "ERROR: Index " << i << " out of range 3. Use .dt to access time\n";
            exit(-1);
        }
    }
};

namespace franka_example_controllers {

    class ExuCobotCartesianVelocityController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaVelocityCartesianInterface,
            franka_hw::FrankaStateInterface> {
    public:

        // ~= constructor
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;

        // called once per iteration
        void update(const ros::Time &, const ros::Duration &period) override;

        // called once at beginning
        void starting(const ros::Time &) override;

        // called once at end
        void stopping(const ros::Time &) override;

    private:

        ros::Time logTime_;
        ros::Duration elapsedTime_;

        std::string logTimeString_;     // string with time starting at beginning of program
        std::string rosTimeString_;     // string with time starting at ros::Time(0)

        // callback method: is called when new command from Exudyn arrives
        void updateReferencePoseCallback(const util::segmentCommand &msg);

        // updates trajectory -> creates new segment when previous is finished
        void updateTrajectory();

#if ENABLE_LOGGING
        // various logging functionalities

        void logEvaluatedTrajectory();
        void logCurrentPosition(const std::array<double, 16> &current_pose, const std::array< double, 7 > &current_joint_positions);
        void logTrajectoryCreation(const SamplingPoint3 &startState, const SamplingPoint3 &endState);
        void logCoefficients();

        std::shared_ptr<TextLogger> textLogger_;
        std::shared_ptr<CsvLogger> evalTrajLogger_; // logger for evalueated trajectory
        LogThreader logThreader_;   // thread handler for multiple loggers

        // file for text logging
        std::ofstream generalLogFile_;

        // various files for logging values
        std::ofstream referenceLogFile_;
        std::ofstream commandLogFile_;
        std::ofstream evaluatedTrajectoryFile_;
        std::ofstream currentPositionFile_;
        std::ofstream trajectoryCreationFile_;
        std::ofstream coefficientsFile_;
        std::ofstream trajectoryCreationFile2_;

        const bool logYonly_ = false;
#endif

        // some constants
        const int polynomialDegree_ = 5;
        const int minimalCommandBufferSize_ = 4;
        const bool exitIfTheoreticalValuesExceedLimits_ = true;
        const bool exitIfCommandBufferEmpty_ = false;

        // if controller can't create new trajectory segment when previous is finished, overdueTime gets accumulated and must be recovered afterwards
        ros::Duration overdueTime_;

        // max v,a,j values according to https://frankaemika.github.io/docs/control_parameters.html#limit-table
        const double maxV_trans_ = 1.7; // m/s
        const double maxA_trans_ = 13.0; // m/s²
        const double maxJ_trans_ = 6500.0; // m/s³

        // called from updateTrajectory; calculates polynomial coefficients for single axis
        std::vector<double> calcCoefs(SamplingPoint startState, SamplingPoint endState, double T);
        // evaluates polynomial at given point of time
        SamplingPoint evaluatePolynomial(std::vector<double> &coef, double t);

        franka_hw::FrankaVelocityCartesianInterface *velocityCartesianInterface_;
        std::unique_ptr <franka_hw::FrankaCartesianVelocityHandle> velocityCartesianHandle_;

        SamplingPoint3 currentState_;    // pos, vel, acc and jerk for x, y, and z; describing current state of trajectory
        std::vector<std::vector<double>> coefs_;    // trajectory / polynom coefficients

        std::array<double, 6> currentCommand_;
        std::array<double, 6> lastCommand_;

        std::vector<Command> commandBuffer_;              // x, y and z value of next positions to travers
        const int commandBufferLength_ = 50;             // length of position buffer. Also if buffer is vector, it's length is static
        // as position_buffer will be a ring buffer, current indices for reading and writing have to be stored
        int commandBufferWritingIndex_;                 // holds index in which to write next (write then increase)
        int commandBufferReadingIndex_;                 // holds index from which to read next (read then increase)
        const int getCommandBufferReserve();               // returns amount of stored next positions

        bool allowDrift_ = true;           // true: if robot drifts away from reference path because of latencies, it will not try to get back to it

        void publishState(ros::Time now, const SamplingPoint3 &state);  // method to publish current kinematic state to ROS topic

        std::vector<double> currentReference__;    // only for analytics

        ros::Duration segmentDuration_;               // planned duration of one segment in s
        ros::Duration segmentTime_;                   // time in current segment in s

        // ROS publishers
        ros::Publisher pubCurrentReferenceConfirmation_;    // publisher for sending received reference back
        ros::Publisher pubCommandedVelocity_;     // publisher for current commanded velocity
        ros::Publisher pubCurrentTrajectoryPos_; // publisher for current trajectory position
        ros::Publisher pubCurrentPose_;   // publisher for current pose
        ros::Publisher pubCurrentState_;  // publisher for full kinematic state

        ros::Subscriber subDesiredPose_;  // Subscriber for new desired pose
    };

}  // namespace franka_example_controllers
