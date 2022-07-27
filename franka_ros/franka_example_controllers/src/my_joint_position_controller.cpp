// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/my_joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

    bool MyJointPositionController::init(hardware_interface::RobotHW *robot_hardware,
                                         ros::NodeHandle &node_handle) {
        position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
        if (position_joint_interface_ == nullptr) {
            ROS_ERROR(
                    "JointPositionExampleController: Error getting position joint interface from hardware!");
            return false;
        }
        std::vector <std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names)) {
            ROS_ERROR("JointPositionExampleController: Could not parse joint names");
        }
        if (joint_names.size() != 7) {
            ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                                     << joint_names.size() << " instead of 7 names!");
            return false;
        }
        position_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            try {
                position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
            } catch (const hardware_interface::HardwareInterfaceException &e) {
                ROS_ERROR_STREAM(
                        "JointPositionExampleController: Exception getting joint handles: " << e.what());
                return false;
            }
        }

        /*std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        for (size_t i = 0; i < q_start.size(); i++) {
          if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
            ROS_ERROR_STREAM(
                "JointPositionExampleController: Robot is not in the expected starting position for "
                "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
                "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
            return false;
          }
        }*/

        return true;
    }

    void MyJointPositionController::starting(const ros::Time & /* time */) {
        for (size_t i = 0; i < 7; ++i) {
            initial_pose_[i] = position_joint_handles_[i].getPosition();
        }

        elapsed_time_ = ros::Duration(0.0);
    }

    template<typename T>
    void printArray7(std::array<T, 7> array) {
        for (size_t i = 0; i < 7; ++i) {
            std::cout << array[i] << "\t";
        }
        std::cout << std::endl;
    }

    void MyJointPositionController::update(const ros::Time & /*time*/,
                                           const ros::Duration &period) {
        elapsed_time_ += period;

        //double target_pose[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        //double target_pose[] = {0.0, -0.22449781786768058, 0.0, -2.6762069058915854, 0.0, 2.4217752625883384, 0.7664495172173067};
        double target_pose[] = {-1.93489204e-03, -2.28897689e-01, 2.02668799e-03, -2.69957050e+00, 7.39679528e-04, 2.47067313e+00, 7.84857743e-01};

        std::array<double, 7> current_pose{};
        std::array<double, 7> error{};
        std::array<int, 7> direction{};

        for (size_t i = 0; i < 7; ++i) {
            current_pose[i] = position_joint_handles_[i].getPosition();
            error[i] = target_pose[i] - current_pose[i];

            if (error[i] > 0) direction[i] = 1;
            else if (error[i] < 0) direction[i] = -1;
            else direction[i] = 0;
        }

        for (size_t i = 0; i < 7; ++i) {
            double newPos;
            double stepSize = 0.0015;
            if(abs(error[i]) > stepSize * 2) {
                newPos = current_pose[i] + stepSize * direction[i];
            }
            else{
                newPos = current_pose[i];
            }
            position_joint_handles_[i].setCommand(newPos);
        }

        /*
        double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;
        std::cout << delta_angle << "\t\t" << delta_angle - last_angle_ << std::endl;
        last_angle_ = delta_angle;
        for (size_t i = 0; i < 7; ++i) {
            double newPos;
            if (i == 4) {
                newPos = initial_pose_[i] - delta_angle;
            } else {
                newPos = initial_pose_[i] + delta_angle;
            }
            position_joint_handles_[i].setCommand(newPos);
        }
        */

    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyJointPositionController,
        controller_interface::ControllerBase
)
