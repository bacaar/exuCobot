// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/my_cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

    bool MyCartesianVelocityController::init(hardware_interface::RobotHW *robot_hardware,
                                             ros::NodeHandle &node_handle) {
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("MyCartesianVelocityController: Could not get parameter arm_id");
            return false;
        }

        velocity_cartesian_interface_ = robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
        if (velocity_cartesian_interface_ == nullptr) {
            ROS_ERROR(
                    "MyCartesianVelocityController: Could not get Cartesian velocity interface from "
                    "hardware");
            return false;
        }

        try {
            velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
                    velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM(
                    "MyCartesianVelocityController: Exception getting Cartesian handle: " << e.what());
            return false;
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("MyCartesianVelocityController: Could not get state interface from hardware");
            return false;
        }

        try {
          auto state_handle = state_interface->getHandle(arm_id + "_robot");
        } catch (const hardware_interface::HardwareInterfaceException& e) {
          ROS_ERROR_STREAM(
              "MyCartesianVelocityController: Exception getting state handle: " << e.what());
          return false;
        }

        // set callback method for updating target pose
        sub_desired_pose_ = node_handle.subscribe("setTargetPose", 20,
                                                  &MyCartesianVelocityController::updateTargetPoseCallback, this,
                                                  ros::TransportHints().reliable().tcpNoDelay());

        // create publisher for current target pose
        pub_current_target_ = node_handle.advertise<geometry_msgs::PoseStamped>("getCurrentTarget", 20);

        // create publisher for current pose
        pub_current_trajectory_ = node_handle.advertise<geometry_msgs::PoseStamped>("getEvaluatedTrajectory", 20);

        // create publisher for current pose
        pub_current_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("getCurrentPose", 20);

        // initialize variables
        current_state_ = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));
        current_target_ = std::vector<double>(3, 0);
        position_buffer_ = std::vector<std::vector<double>>(position_buffer_length_, std::vector<double>(3, 0));
        position_buffer_index_reading_ = 0;
        position_buffer_index_writing_ = 1;
        coefs_ = std::vector<std::vector<double>>(3, std::vector<double>(6, 0));

        return true;
    }

    void MyCartesianVelocityController::starting(const ros::Time & /* time */) {
        auto initial_pose = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

        // set next positions on current positions to stay here (NOT ZERO!!!)
        // TODO can I let this on zero because of my brilliant ring buffer logic?

        // set position entries of current_state_ vector to current positions
        current_state_[0][0] = initial_pose[12];
        current_state_[1][0] = initial_pose[13];
        current_state_[2][0] = initial_pose[14];

        segment_time_ = 0;

        elapsed_time_ = ros::Duration(0.0);

        if(testing_){
            std::cerr << "WARNING: Testing mode active!\n";
        }
    }

    const int MyCartesianVelocityController::getPositionBufferReserve(){

        if (position_buffer_index_writing_ > position_buffer_index_reading_){
            return position_buffer_index_writing_ - position_buffer_index_reading_ - 1;
        }
        else if (position_buffer_index_writing_ < position_buffer_index_reading_){
            return position_buffer_index_writing_ + position_buffer_length_ - position_buffer_index_reading_- 1;
        }
        else{
            std::cerr << "ERROR: Writing index has catched reading index!";
            exit(-1);
        }
    }

    std::vector<double> MyCartesianVelocityController::calcCoefs(double s0, double ds0, double dds0, double sT, double dsT, double ddsT, double T){
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

    std::vector<double> MyCartesianVelocityController::evaluatePolynom(std::vector<double> &coef, double t){

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        std::vector<double> stateVec(3, 0);

        stateVec[0] =    coef[0]*t5 +    coef[1]*t4 +   coef[2]*t3 +   coef[3]*t2 + coef[4]*t + coef[5];
        stateVec[1] =  5*coef[0]*t4 +  4*coef[1]*t3 + 3*coef[2]*t2 + 2*coef[3]*t  + coef[4];
        stateVec[2] = 20*coef[0]*t3 + 12*coef[1]*t2 + 6*coef[2]*t  + 2*coef[3];

        return stateVec;

    }

    void MyCartesianVelocityController::update(const ros::Time & /* time */,
                                           const ros::Duration &period) {
        elapsed_time_ += period;
        segment_time_ += period.toSec();

        //std::cerr << "Update\t pos buffer write : " << position_buffer_index_writing_ << "\t read: " << position_buffer_index_reading_ << "\treserve: " << getPositionBufferReserve() << std::endl;

        static bool started = false;
        static int counter = 0;

        // get current pose
        std::array<double, 16> current_pose = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

        // when starting the controller, wait until position buffer is filled with 3 values
        if(!started){
            if(getPositionBufferReserve() >= 8){
                started = true;
                std::cerr << "Buffer partly filled with " << getPositionBufferReserve() << " entries. Starting-permission granted.\n";
            }
        }

        bool newStatePublished = false;
        int max_segments = 0;

        if(started){

            // if segment_duration_ has passed, calc new trajectory
            if(segment_time_ >= segment_duration_){
                if(getPositionBufferReserve() >= 2){
                    //std::cerr << period.toSec() << "\t" << segment_time_ << std::endl;

                    // at last iteration segment_time was at e.g. 0.009 (if segment_duration_ is 0.01)
                    // last calculation for previous segment has still to be done in order to let new segment start from correct position
                    current_state_[0] = evaluatePolynom(coefs_[0], segment_time_);
                    current_state_[1] = evaluatePolynom(coefs_[1], segment_time_);
                    current_state_[2] = evaluatePolynom(coefs_[2], segment_time_);

                    updateTrajectory();

                    if(max_segments != 0 && ++counter > max_segments){
                        std::cerr << "DEBUG BREAK: " << max_segments << " segments completed!\n";
                        exit(-1);
                    }

                    // reset segment_time_ as new one starts now
                    // normally subtracting duration once should be enough
                    while(segment_time_ > segment_duration_){
                        segment_time_ -= segment_duration_;
                    }
                }
                else { // if there are not enough values to update trajectory (2 needed), stay at position
                    std::cerr << "ERROR: Not enough positions to calculate new trajectory segment!\n";
                    exit(-1);
                }
            }

            // if within semgent_duration, calc new state
            if(segment_time_ <= segment_duration_){
                // calculat new positions, velocities and accelerations
                current_state_[0] = evaluatePolynom(coefs_[0], segment_time_);
                current_state_[1] = evaluatePolynom(coefs_[1], segment_time_);
                current_state_[2] = evaluatePolynom(coefs_[2], segment_time_);

                // when debugging
                if (max_segments != 0){
                    //std::cerr << "[" << current_state_[1][0] << ", " << current_state_[1][1] << ", " << current_state_[1][2] << ", " << elapsed_time_.toSec() << "],\n";
                }

                // compose new velocity
                double vx = current_state_[0][1];
                double vy = current_state_[1][1];
                double vz = current_state_[2][1];

                double wx, wy, wz;
                wx = wy = wz = 0;

                // update command
                current_command_ = {vx, vy, vz, wx, wy, wz};

                // pass new pose to robot control
                velocity_cartesian_handle_->setCommand(current_command_);
                newStatePublished = true;
            }
            else{
                std::cerr << "ERROR: semgment_time > segment_duration\n";
                exit(-1);
            }
        }

        if(!newStatePublished) { // if no new position has been calulated, stay at position
            velocity_cartesian_handle_->setCommand(current_command_);
        }

        // publish positions for python analytics
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();

        // current target pos
        //msg.pose.position.x = current_target_[0];
        //msg.pose.position.y = current_target_[1];
        //msg.pose.position.z = current_target_[2];
        //pub_current_target_.publish(msg);

        // current trajectory val
        msg.pose.position.x = current_state_[0][0];
        msg.pose.position.y = current_state_[1][0];
        msg.pose.position.z = current_state_[2][0];
        pub_current_trajectory_.publish(msg);

        // current pos
        msg.pose.position.x = current_pose[12];
        msg.pose.position.y = current_pose[13];
        msg.pose.position.z = current_pose[14];
        pub_current_pose_.publish(msg);
    }

    void MyCartesianVelocityController::updateTargetPoseCallback(const geometry_msgs::PoseStamped &msg) {

        // send it back immediately
        geometry_msgs::PoseStamped msgnew = msg;
        msgnew.header.stamp = ros::Time::now();
        pub_current_target_.publish(msgnew);

        if(position_buffer_index_writing_ == position_buffer_index_reading_){
            std::cerr << "Position buffer full!\n";
            exit(-1);
        }

        position_buffer_[position_buffer_index_writing_] = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
        int old = position_buffer_index_writing_;
        position_buffer_index_writing_ = (position_buffer_index_writing_ + 1) % position_buffer_length_;

        // for analytics
        current_target_[0] = msg.pose.position.x;
        current_target_[1] = msg.pose.position.y;
        current_target_[2] = msg.pose.position.z;
    }

    void MyCartesianVelocityController::updateTrajectory(){

        // get current pose
        std::array<double, 16> current_pose = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

        // index of next positions
        int i1 = (position_buffer_index_reading_ + 1) % position_buffer_length_; // next position
        int i2 = (position_buffer_index_reading_ + 2) % position_buffer_length_; // second next position

        // calculate desired velocity for end of (first) segment as mean velocity of next two segments
        std::array<double, 3> next_velocity{};

        if(testing_){
            next_velocity[0] = (position_buffer_[i2][0] - current_state_[0][0])/(segment_duration_*2);
            next_velocity[1] = (position_buffer_[i2][1] - current_state_[1][0])/(segment_duration_*2);
            next_velocity[2] = (position_buffer_[i2][2] - current_state_[2][0])/(segment_duration_*2);
        }
        else{
            next_velocity[0] = (position_buffer_[i2][0] - current_pose[12])/(segment_duration_*2);
            next_velocity[1] = (position_buffer_[i2][1] - current_pose[13])/(segment_duration_*2);
            next_velocity[2] = (position_buffer_[i2][2] - current_pose[14])/(segment_duration_*2);
        }


        // calculate polynom coefficients
        // TODO: using current velocity and acceleration from current_state_ vector is not 100% correct, as they are the vel and acc from last step

        // std::cerr are for evaluating trajectory in pyhton, to control if calculation is correct
        if(testing_){
            coefs_[0] = calcCoefs(current_state_[0][0], current_state_[0][1], current_state_[0][2], position_buffer_[i1][0], next_velocity[0], 0, segment_duration_);
            coefs_[1] = calcCoefs(current_state_[1][0], current_state_[1][1], current_state_[1][2], position_buffer_[i1][1], next_velocity[1], 0, segment_duration_);
            coefs_[2] = calcCoefs(current_state_[2][0], current_state_[2][1], current_state_[2][2], position_buffer_[i1][2], next_velocity[2], 0, segment_duration_);
            //std::cerr << "[[" << current_state_[1][0];
        }
        else{
            coefs_[0] = calcCoefs(current_pose[12], current_state_[0][1], current_state_[0][2], position_buffer_[i1][0], next_velocity[0], 0, segment_duration_);
            coefs_[1] = calcCoefs(current_pose[13], current_state_[1][1], current_state_[1][2], position_buffer_[i1][1], next_velocity[1], 0, segment_duration_);
            coefs_[2] = calcCoefs(current_pose[14], current_state_[2][1], current_state_[2][2], position_buffer_[i1][2], next_velocity[2], 0, segment_duration_);
            //std::cerr << "[[" << current_pose[13];
        }

        /*std::cerr<< ", " << current_state_[1][1] << ", " << current_state_[1][2];
        std::cerr << ", " << position_buffer_[i1][1] << ", " << next_velocity[1] << ", " << 0 << ", " << segment_duration_ << "],\t\t\t";
        std::cerr << "[";
        for(int i = 0; i < 6; ++i){
            std::cerr << coefs_[1][i];
            if (i != 5) std::cerr << ", ";
        }
        std::cerr << "]]," << std::endl;
         */

        // for next segment
        int old = position_buffer_index_reading_;
        position_buffer_index_reading_ = (position_buffer_index_reading_ + 1) % position_buffer_length_;
    }

    void MyCartesianVelocityController::stopping(const ros::Time & /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyCartesianVelocityController,
        controller_interface::ControllerBase
)
