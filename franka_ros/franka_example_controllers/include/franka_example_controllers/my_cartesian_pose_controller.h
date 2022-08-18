// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <geometry_msgs/PoseStamped.h>

#include <franka_hw/franka_cartesian_command_interface.h>

namespace franka_example_controllers {

    class MyCartesianPoseController
            : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                    franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;

        void starting(const ros::Time &) override;

        void update(const ros::Time &, const ros::Duration &period) override;

    private:

        void updateTargetPoseCallback(const geometry_msgs::PoseStamped &msg);

        void updateTrajectory();

        franka_hw::FrankaPoseCartesianInterface *cartesian_pose_interface_;
        std::unique_ptr <franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
        ros::Duration elapsed_time_;
        std::array<double, 16> initial_pose_{};
        //std::array<double, 16> desired_pose_{};

        std::vector<std::vector<double>> current_state_;    // pos, vel and acc for x, y, and z; describing current state of trajectory
        std::vector<std::vector<double>> coefs_;    // trajectory / polynom coefficients

        std::vector<std::vector<double>> position_buffer_;  // x, y and z value of next positions to travers
        const int position_buffer_length_ = 50;             // length of position buffer. Also if buffer is vector, it's length is static
        // as position_buffer will be a ring buffer, current indices for reading and writing have to be stored
        int position_buffer_index_writing_;                 // holds index in which to write next (write then increase)
        int position_buffer_index_reading_;                 // holds index from which to read next (read then increase)
        const int getPositionBufferReserve();               // returns amount of stored next positions

        std::vector<double> current_target_;    // only for analytics

        const double segment_duration_ = 0.01;  // planned duration of one segment in s
        double segment_time_;                   // time in current segment in s  

        const bool testing_ = false;        // flag to use current_state_ instead of current_pose_

        ros::Publisher pub_current_target_; // publisher for current registered target position
        ros::Publisher pub_current_trajectory_; // publisher for current trajectory value
        ros::Publisher pub_current_pose_;   // publisher for current pose

        ros::Subscriber sub_desired_pose_;  // Subscriber for new desired pose
    };

}  // namespace franka_example_controllers
