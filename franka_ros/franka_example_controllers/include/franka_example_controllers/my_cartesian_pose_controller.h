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

        void updateDesiredPoseCallback(const geometry_msgs::PoseStamped &msg);

        void updateTrajectory(double x, double y, double z);

        franka_hw::FrankaPoseCartesianInterface *cartesian_pose_interface_;
        std::unique_ptr <franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
        ros::Duration elapsed_time_;
        std::array<double, 16> initial_pose_{};
        //std::array<double, 16> desired_pose_{};

        std::vector<std::vector<double>> current_state_;
        std::vector<double> current_target_;    // only for analytics
        std::vector<double> next_position_;
        std::vector<double> second_next_position_;
        std::vector<std::vector<double>> coefs_;

        const double segment_duration_ = 0.01;  // planned duration of one segment in s
        double segment_time_;                   // time in current segment in s  

        ros::Publisher pub_current_pose_;   // publisher for current pose
        ros::Publisher pub_current_target_; // publisher for current registered target position

        ros::Subscriber sub_desired_pose_;  // Subscriber for new desired pose
    };

}  // namespace franka_example_controllers
