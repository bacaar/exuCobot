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
#include <util/kinematicState3dStamped.h>

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

namespace franka_example_controllers {

    class MyCartesianVelocityController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaVelocityCartesianInterface,
            franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;

        void update(const ros::Time &, const ros::Duration &period) override;

        void starting(const ros::Time &) override;

        void stopping(const ros::Time &) override;

    private:

        ros::Time logTime_;
        ros::Duration elapsed_time_;

        void updateTargetPoseCallback(const geometry_msgs::PoseStamped &msg);

        void updateTrajectory();

        void logEvaluatedTrajectory();
        void logCurrentPosition(const std::array<double, 16> &current_pose);
        void logTrajectoryCreation(const State3 &startState, const State3 &endState);
        void logCoefficients();

        std::ofstream generalLogFile_;
        std::ofstream commandLogFile_;
        std::ofstream evaluatedTrajectoryFile_;
        std::ofstream currentPositionFile_;
        std::ofstream trajectoryCreationFile_;
        std::ofstream coefficientsFile_;

        const int polynomialDegree_ = 5;
        const int nominalPositionBufferSize_ = 8;
        const bool useActualRobotPosition_ = true;        // flag to use current_pose_ (true) instead of current_state_ (false)
        const bool exitIfTheoreticalValuesExceedLimits_ = true;

        const bool logYonly_ = false;

        std::vector<double> calcCoefs(State startState, State endState, double T);
        State evaluatePolynom(std::vector<double> &coef, double t);

        franka_hw::FrankaVelocityCartesianInterface *velocity_cartesian_interface_;
        std::unique_ptr <franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;

        State3 current_state_;    // pos, vel, acc and jerk for x, y, and z; describing current state of trajectory
        std::vector<std::vector<double>> coefs_;    // trajectory / polynom coefficients

        std::array<double, 6> current_command_;
        std::array<double, 6> last_command_;

        std::vector<std::vector<double>> position_buffer_;  // x, y and z value of next positions to travers
        const int position_buffer_length_ = 50;             // length of position buffer. Also if buffer is vector, it's length is static
        // as position_buffer will be a ring buffer, current indices for reading and writing have to be stored
        int position_buffer_index_writing_;                 // holds index in which to write next (write then increase)
        int position_buffer_index_reading_;                 // holds index from which to read next (read then increase)
        const int getPositionBufferReserve();               // returns amount of stored next positions

        std::vector<double> current_target_;    // only for analytics

        const double segment_duration_ = 0.01;  // planned duration of one segment in s
        double segment_time_;                   // time in current segment in s

        ros::Publisher pub_current_target_confirmation_;    // publisher for sending received target back
        ros::Publisher pub_commanded_velocity_;     // publisher for current commanded velocity
        ros::Publisher pub_current_trajectory_pos_; // publisher for current trajectory position
        ros::Publisher pub_current_pose_;   // publisher for current pose
        ros::Publisher pub_current_state_;  // publisher for full kinematic state

        ros::Subscriber sub_desired_pose_;  // Subscriber for new desired pose
    };

}  // namespace franka_example_controllers
