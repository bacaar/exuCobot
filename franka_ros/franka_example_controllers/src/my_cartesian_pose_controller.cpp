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


        // create publisher for current pose
        pub_current_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("getCurrentPose", 20);

        // create publisher for current target pose
        pub_current_target_ = node_handle.advertise<geometry_msgs::PoseStamped>("getCurrentTarget", 20);

        // initialize variables
        current_state_ = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));
        current_target_ = std::vector<double>(3, 0);
        next_position_ = std::vector<double>(3, 0);
        second_next_position_ = std::vector<double>(3, 0);
        coefs_ = std::vector<std::vector<double>>(3, std::vector<double>(6, 0));

        return true;
    }

    void MyCartesianPoseController::starting(const ros::Time & /* time */) {
        initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

        // set next positions on current positions to stay here (NOT ZERO!!!)
        next_position_[0] = second_next_position_[0] = initial_pose_[12];
        next_position_[1] = second_next_position_[1] = initial_pose_[13];
        next_position_[2] = second_next_position_[2] = initial_pose_[14];

        // set position entries of current_state_ vector to current positions
        current_state_[0][0] = initial_pose_[12];
        current_state_[1][0] = initial_pose_[13];
        current_state_[2][0] = initial_pose_[14];

        segment_time_ = 0;

        elapsed_time_ = ros::Duration(0.0);
    }


    std::vector<double> calcCoefs(double s0, double ds0, double dds0, double sT, double dsT, double ddsT, double T){
        double T2 = T*T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;

        std::vector<double> solution(6, 0);

        solution[0] = -dds0/(2*T3) + ddsT/(2*T3) - 3*ds0/T4 - 3*dsT/T4 - 6*s0/T5 + 6*sT/T5;
        solution[1] = 3*dds0/(2*T2) - ddsT/T2 + 8*ds0/T3 + 7*dsT/T3 + 15*s0/T4 - 15*sT/T4;
        solution[2] = -3*dds0/(2*T) + ddsT/(2*T) - 6*ds0/T2 - 4*dsT/T2 - 10*s0/T3 + 10*sT/T3;
        solution[3] = dds0/2;
        solution[4] = ds0;
        solution[5] = s0;

        return solution;
    }

    std::vector<double> evaluatePolynom(std::vector<double> &coef, double t){

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        std::vector<double> stateVec(3, 0);

        stateVec[0] =    coef[0]*t5 +    coef[1]*t4 +   coef[2]*t3 +   coef[3]*t2 + coef[4]*t + coef[5];
        stateVec[1] =  5*coef[0]*t4 +  4*coef[1]*t3 + 3*coef[2]*t2 + 2*coef[3]*t  + coef[4];
        stateVec[2] = 20*coef[0]*t3 + 12*coef[1]*t2 + 6*coef[3]*t  + 2*coef[4];

        return stateVec;

    }

    void MyCartesianPoseController::update(const ros::Time & /* time */,
                                           const ros::Duration &period) {
        elapsed_time_ += period;
        segment_time_ += period.toSec();

        // calculat new positions, velocities and accelerations
        current_state_[0] = evaluatePolynom(coefs_[0], segment_time_);
        current_state_[1] = evaluatePolynom(coefs_[1], segment_time_);
        current_state_[2] = evaluatePolynom(coefs_[2], segment_time_);

        // compose new pose
        std::array<double, 16> new_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        new_pose[12] = current_state_[0][0];
        new_pose[13] = current_state_[1][0];
        new_pose[14] = current_state_[2][0];

        // pass new pose to robot control
        cartesian_pose_handle_->setCommand(new_pose);

        // publish positions for python analytics
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();

        // current pos
        msg.pose.position.x = current_state_[0][0];
        msg.pose.position.y = current_state_[1][0];
        msg.pose.position.z = current_state_[2][0];
        pub_current_pose_.publish(msg);

        // current target pos
        msg.pose.position.x = current_target_[0];
        msg.pose.position.y = current_target_[1];
        msg.pose.position.z = current_target_[2];
        pub_current_target_.publish(msg);
    }

    void MyCartesianPoseController::updateDesiredPoseCallback(const geometry_msgs::PoseStamped &msg) {

        //// For the moment modify position only, leave orientation unchanged
        updateTrajectory(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        
        // for analytics
        current_target_[0] = msg.pose.position.x;
        current_target_[1] = msg.pose.position.y;
        current_target_[2] = msg.pose.position.z;
    }

    void MyCartesianPoseController::updateTrajectory(double targetX, double targetY, double targetZ){

        // get current pose
        std::array<double, 16> current_pose = cartesian_pose_handle_->getRobotState().O_T_EE_d;

        // set second next position to (first) next position
        next_position_[0] = second_next_position_[0];
        next_position_[1] = second_next_position_[1];
        next_position_[2] = second_next_position_[2];

        // load new second next position
        second_next_position_[0] = targetX;
        second_next_position_[1] = targetY;
        second_next_position_[2] = targetZ;

        // calculate desired velocity for end of (first) segment as mean velocity of next two segments
        std::array<double, 3> next_velocity{};
        next_velocity[0] = (second_next_position_[0] - current_pose[12])/segment_duration_;
        next_velocity[1] = (second_next_position_[1] - current_pose[13])/segment_duration_;
        next_velocity[2] = (second_next_position_[2] - current_pose[14])/segment_duration_;

        // calculate polynom coefficients
        // TODO: using current velocity and acceleration from current_state_ vector is not 100% correct, as they are the vel and acc from last step
        coefs_[0] = calcCoefs(current_pose[12], current_state_[0][1], current_state_[0][3], next_position_[0], next_velocity[0], 0, segment_duration_);
        coefs_[1] = calcCoefs(current_pose[13], current_state_[1][1], current_state_[1][3], next_position_[1], next_velocity[1], 0, segment_duration_);
        coefs_[2] = calcCoefs(current_pose[14], current_state_[2][1], current_state_[2][3], next_position_[2], next_velocity[2], 0, segment_duration_);

        // reset segment_time_ as new one starts now
        segment_time_ = 0;

    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyCartesianPoseController,
        controller_interface::ControllerBase
)
