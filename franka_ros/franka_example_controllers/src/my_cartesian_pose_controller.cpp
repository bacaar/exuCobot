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

        //maxVel_ = 0.005; // m/sec
        maxVel_ = 0.00001; // m/step
        maxStepSize_ = 0.00001;

        return true;
    }

    void MyCartesianPoseController::starting(const ros::Time & /* time */) {
        stepSizeX_ = 0;
        stepSizeY_ = 0;
        stepSizeZ_ = 0;
        elapsed_time_ = ros::Duration(0.0);
    }

    void MyCartesianPoseController::update(const ros::Time & /* time */,
                                           const ros::Duration &period) {
        elapsed_time_ += period;

        updateTargetPosition(0.7, 0.0, 0.3);

        std::array<double, 16> new_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        //std::cout << "oldX " << new_pose[12] << "\toldY " << new_pose[13] << "\toldZ " << new_pose[14] << std::endl;
        new_pose[12] += stepSizeX_;
        new_pose[13] += stepSizeY_;
        new_pose[14] += stepSizeZ_;
        //std::cout << "newX " << new_pose[12] << "\tnewY " << new_pose[13] << "\tnewZ " << new_pose[14] << std::endl;

        // rough collision check for robot base and table
        if (new_pose[12] < 0.15 && std::abs(new_pose[13]) < 0.15 && new_pose[14] < 0.05){
            std::cerr << "rudimentary collision check failed!" << std::endl;
            exit(-1);
        }

        cartesian_pose_handle_->setCommand(new_pose);
    }

    void MyCartesianPoseController::updateDesiredPoseCallback(const geometry_msgs::PoseStamped &msg) {

        //// For the moment modify position only, leave orientation unchanged
        updateTargetPosition(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    }

    void MyCartesianPoseController::updateTargetPosition(double targetX, double targetY, double targetZ){

        // get current pose
        std::array<double, 16> current_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;

        // total difference to desired position
        double deltaX = targetX - current_pose[12];
        double deltaY = targetY - current_pose[13];
        double deltaZ = targetZ - current_pose[14];

        std::cout << "dx=" << deltaX << "\tdy=" << deltaY << "\tdz=" << deltaZ << std::endl;

        // robot moves with maxvel on axis, on which difference is biggest. The other two axes have to slow down
        double maxDiff = std::abs(deltaX) > std::abs(deltaY) ? std::abs(deltaX) : std::abs(deltaY);
        maxDiff = std::abs(deltaZ) > maxDiff ? std::abs(deltaZ) : maxDiff;

        int neededSteps = (int)ceil(maxDiff/maxStepSize_);
        std::cout << "maxDiff=" << maxDiff << "\tmaxStepSize=" << maxStepSize_ << "\tneededSteps=" << neededSteps << std::endl;

        stepSizeX_ = 0;
        stepSizeY_ = 0;
        stepSizeZ_ = 0;

        if (neededSteps > 0) {
            stepSizeX_ = deltaX / neededSteps;
            stepSizeY_ = deltaY / neededSteps;
            stepSizeZ_ = deltaZ / neededSteps;
        }

        std::cout << "sx=" << stepSizeX_ << "\tsy=" << stepSizeY_ << "\tsz=" << stepSizeZ_ << std::endl;
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyCartesianPoseController,
        controller_interface::ControllerBase
)
