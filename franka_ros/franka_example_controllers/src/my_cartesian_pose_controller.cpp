// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/my_cartesian_pose_controller.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

    bool MyCartesianPoseController::init(hardware_interface::RobotHW *robot_hardware,
                                         ros::NodeHandle &node_handle) {
        cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
        if (cartesian_pose_interface_ == nullptr) {
            ROS_ERROR(
                    "MyCartesianPoseController: Could not get Cartesian Pose "
                    "interface from hardware");
            return false;
        }

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("MyCartesianPoseController: Could not get parameter arm_id");
            return false;
        }

        try {
            cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
                    cartesian_pose_interface_->getHandle(arm_id + "_robot"));
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM(
                    "MyCartesianPoseController: Exception getting Cartesian handle: " << e.what());
            return false;
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("MyCartesianPoseController: Could not get state interface from hardware");
            return false;
        }

        try {
            auto state_handle = state_interface->getHandle(arm_id + "_robot");

            std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            /*for (size_t i = 0; i < q_start.size(); i++) {
              if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
                ROS_ERROR_STREAM(
                    "MyCartesianPoseController: Robot is not in the expected starting position for "
                    "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
                    "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
                return false;
              }
            }*/
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM(
                    "MyCartesianPoseController: Exception getting state handle: " << e.what());
            return false;
        }

        // set callback method for updating desired pose
        sub_desired_pose_ = node_handle.subscribe("setDesiredPose", 20,
                                                  &MyCartesianPoseController::updateDesiredPoseCallback, this,
                                                  ros::TransportHints().reliable().tcpNoDelay());

        return true;
    }

    void MyCartesianPoseController::starting(const ros::Time & /* time */) {
        initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        desired_pose_ = initial_pose_;
        elapsed_time_ = ros::Duration(0.0);
    }

    void MyCartesianPoseController::update(const ros::Time & /* time */,
                                           const ros::Duration &period) {
        elapsed_time_ += period;

        // for circle creation
        /*
        double xc = 0.5;
        double yc = 0.0;
        double zc = 0.3;
        double radius = 0.2;

        double angle = elapsed_time_.toSec() * 0.5;
        double x = xc + radius * std::sin(angle);
        double y = yc + radius * std::cos(angle);
        double z = zc;
        std::array<double, 16> new_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        new_pose[12] += x;
        new_pose[13] += y;
        new_pose[14] += z;

        cartesian_pose_handle_->setCommand(new_pose);
         */


        /*
        double radius = 0.3;
        double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
        double delta_x = radius * std::sin(angle);
        double delta_y = radius * std::cos(angle);
        std::array<double, 16> new_pose = initial_pose_;
        new_pose[12] -= delta_x;
        new_pose[13] -= delta_y;
        cartesian_pose_handle_->setCommand(new_pose);
         */


        std::array<double, 16> pose_current = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        double delta_x = 0.00001;   //(scale down to slow the movement)
        pose_current[12] += delta_x;
        std::cout << pose_current[12] << std::endl;
        cartesian_pose_handle_->setCommand(pose_current);
    }

    void MyCartesianPoseController::updateDesiredPoseCallback(const geometry_msgs::PoseStamped &msg) {

        // for the moment: leave orientation as it is, move only positional
        desired_pose_[12] = msg.pose.position.x;
        desired_pose_[13] = msg.pose.position.y;
        desired_pose_[14] = msg.pose.position.z;

    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyCartesianPoseController,
        controller_interface::ControllerBase
)
