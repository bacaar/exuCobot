// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/my_cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>

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

        if(polynomialDegree_ != 3 && polynomialDegree_ != 5){
            std::cerr << "ERROR: Polynomial degree for interpolation not implemented!\n";
            exit(-1);
        }

        // create publisher for returning target pose
        pub_current_target_confirmation_ = node_handle.advertise<geometry_msgs::PoseStamped>("getCurrentTarget", 20);

        // create publisher for current command
        pub_commanded_velocity_ = node_handle.advertise<geometry_msgs::Vector3Stamped>("getCommandedVelocity", 20);

        // create publisher for current pose
        pub_current_trajectory_pos_ = node_handle.advertise<geometry_msgs::Vector3Stamped>("getEvaluatedTrajectory", 20);

        // create publisher for current pose
        pub_current_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("getCurrentPose", 20);

        // create publisher for current state
        pub_current_state_ = node_handle.advertise<util::kinematicState3dStamped>("getCurrentState", 20);

        // initialize variables
        //current_state_ = State3();    // for 3 dimensions a vector of size 4 (s, v, a, j)
        current_target_ = std::vector<double>(3, 0);    // (x, y, z)
        position_buffer_ = std::vector<std::vector<double>>(position_buffer_length_, std::vector<double>(3, 0));
        position_buffer_index_reading_ = 0;
        position_buffer_index_writing_ = 1;
        coefs_ = std::vector<std::vector<double>>(3, std::vector<double>(6, 0));

        std::cout << "INFO: Starting velocity Controller with interpolation polynomial degree " << polynomialDegree_;
        std::cout << " and nominal position buffer size " << nominalPositionBufferSize_ << std::endl;

        return true;
    }

    void MyCartesianVelocityController::starting(const ros::Time & /* time */) {
        std::array<double, 16> initial_pose = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

        // set next positions on current positions to stay here (NOT ZERO!!!)
        // TODO can I let this on zero because of my brilliant ring buffer logic?

        // set position entries of current_state_ vector to current positions
        current_state_.x.pos = initial_pose[12];
        current_state_.y.pos = initial_pose[13];
        current_state_.z.pos = initial_pose[14];

        logTime_ = ros::Time(0);
        segment_time_ = 0;

        elapsed_time_ = ros::Duration(0.0);

        generalLogFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/general.log", std::ios::out);
        commandLogFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/commands.log", std::ios::out);
        evaluatedTrajectoryFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/evaluatedTrajectory.log", std::ios::out);
        currentPositionFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/currentPosition.log", std::ios::out);
        trajectoryCreationFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/trajectoryCreation.log", std::ios::out);
        coefficientsFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/coefficients.log", std::ios::out);

        if(!evaluatedTrajectoryFile_.is_open()) { std::cerr << "WARNING: Could not create open evaluated trajectory log file!\n"; }
        else {
            if(logYonly_) {
                evaluatedTrajectoryFile_ << "s,ns,py,vy,ay,jy\n";
            }
            else{
                evaluatedTrajectoryFile_ << "s,ns,px,vx,ax,jx,py,vy,ay,jy,pz,vz,az,jz\n";
            }
        }

        if(!commandLogFile_.is_open()) { std::cerr << "WARNING: Could not create open evaluated trajectory log file!\n"; }
        else {
            if(logYonly_) {
                commandLogFile_ << "s,ns,vy\n";
            }
            else{
                commandLogFile_ << "s,ns,vx,vy,vz\n";
            }
        }

        if(!currentPositionFile_.is_open()) { std::cerr << "WARNING: Could not create open current position log file!\n"; }
        else {
            if(logYonly_) {
                currentPositionFile_ << "s,ns,py\n";
            }
            else{
                currentPositionFile_ << "s,ns,px,py,pz\n";
            }
        }

        if(!trajectoryCreationFile_.is_open()) { std::cerr << "WARNING: Could not create open trajectory creation log file!\n"; }
        else {
            // cpx = current position x, cvx = current velocity x, etc...
            // npx = next position x, etc...
            // dt = segment_timeobserving position, velocity acceleration and jerk of robot in cartesian and space6
            if(logYonly_) {
                trajectoryCreationFile_ << "s,ns,cpy,cvy,cay,npy,nvy,nay,dt\n";
            }
            else{
                trajectoryCreationFile_ << "s,ns,cpx,cvx,cax,cpy,cvy,cay,cpz,cvz,caz,npx,nvx,nax,npy,nvy,nay,npz,nvz,naz,dt\n";
            }
        }

        if(!coefficientsFile_.is_open()) { std::cerr << "WARNING: Could not create open trajectory creation log file!\n"; }
        else {
                coefficientsFile_ << "s,ns,coord,A,B,C,D,E,F,dt\n";
        }

        if(!useActualRobotPosition_){
            std::cerr << "INFO: Using theoretical trajectory positions!\n";
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

    std::vector<double> MyCartesianVelocityController::calcCoefs(State startState, State endState, double T){
        double T2 = T*T;
        double T3 = T2 * T;

        //double s0, double v0, double a0, double sT, double vT, double aT
        std::vector<double> solution(6, 0);

        if(polynomialDegree_ == 3) {
            solution[0] = 0;    // just for easier implementation afterwards
            solution[1] = 0;
            solution[2] = (startState.vel + endState.vel)/T2 + 2*(startState.pos - endState.pos)/T3;
            solution[3] = (-2*startState.vel - endState.vel)/T + 3*(-startState.pos + endState.pos)/T2;
            solution[4] = startState.vel;
            solution[5] = startState.pos;
            //[ds0/T**2 + dsT/T**2 + 2*s0/T**3 - 2*sT/T**3], [-2*ds0/T - dsT/T - 3*s0/T**2 + 3*sT/T**2], [ds0], [s0]]
        }

        if(polynomialDegree_ == 5) {
            double T4 = T3 * T;
            double T5 = T4 * T;

            solution[0] = -startState.acc / (2 * T3) + endState.acc / (2 * T3) - 3 * startState.vel / T4 - 3 * endState.vel / T4 - 6 * startState.pos / T5 + 6 * endState.pos / T5;
            solution[1] = 3 * startState.acc / (2 * T2) - endState.acc / T2 + 8 * startState.vel / T3 + 7 * endState.vel / T3 + 15 * startState.pos / T4 - 15 * endState.pos / T4;
            solution[2] = -3 * startState.acc / (2 * T) + endState.acc / (2 * T) - 6 * startState.vel / T2 - 4 * endState.vel / T2 - 10 * startState.pos / T3 + 10 * endState.pos / T3;
            solution[3] = startState.acc / 2;
            solution[4] = startState.vel;
            solution[5] = startState.pos;
        }

        return solution;
    }

    State MyCartesianVelocityController::evaluatePolynom(std::vector<double> &coef, double t){

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        State state;

        state.pos  =    coef[0]*t5 +    coef[1]*t4 +   coef[2]*t3 +   coef[3]*t2 + coef[4]*t + coef[5];
        state.vel  =  5*coef[0]*t4 +  4*coef[1]*t3 + 3*coef[2]*t2 + 2*coef[3]*t  + coef[4];
        state.acc  = 20*coef[0]*t3 + 12*coef[1]*t2 + 6*coef[2]*t  + 2*coef[3];
        state.jerk = 60*coef[0]*t2 + 24*coef[1]*t  + 6*coef[2];

        return state;
    }

    void MyCartesianVelocityController::logEvaluatedTrajectory(){

        evaluatedTrajectoryFile_ << logTime_.sec << ",";
        evaluatedTrajectoryFile_ << logTime_.nsec << ",";

        if(!logYonly_) {
            evaluatedTrajectoryFile_ << current_state_.x.pos << ",";
            evaluatedTrajectoryFile_ << current_state_.x.vel << ",";
            evaluatedTrajectoryFile_ << current_state_.x.acc << ",";
            evaluatedTrajectoryFile_ << current_state_.x.jerk << ",";
        }

        evaluatedTrajectoryFile_ << current_state_.y.pos << ",";
        evaluatedTrajectoryFile_ << current_state_.y.vel << ",";
        evaluatedTrajectoryFile_ << current_state_.y.acc << ",";
        evaluatedTrajectoryFile_ << current_state_.y.jerk;

        if(!logYonly_) {
            evaluatedTrajectoryFile_ << ",";
            evaluatedTrajectoryFile_ << current_state_.z.pos << ",";
            evaluatedTrajectoryFile_ << current_state_.z.vel << ",";
            evaluatedTrajectoryFile_ << current_state_.z.acc << ",";
            evaluatedTrajectoryFile_ << current_state_.z.jerk;
        }

        evaluatedTrajectoryFile_ << std::endl;
    }

    void MyCartesianVelocityController::logCurrentPosition(const std::array<double, 16> &current_pose) {
        currentPositionFile_ << logTime_.sec << ",";
        currentPositionFile_ << logTime_.nsec << ",";
        currentPositionFile_ << current_pose[12] << ",";
        currentPositionFile_ << current_pose[13] << ",";
        currentPositionFile_ << current_pose[14];
        currentPositionFile_ << std::endl;
    }

    void MyCartesianVelocityController::logTrajectoryCreation(const State3 &startState, const State3 &endState){

        trajectoryCreationFile_ << logTime_.sec << ",";
        trajectoryCreationFile_ << logTime_.nsec << ",";

        // start state x
        if(!logYonly_) {
            trajectoryCreationFile_ << startState.x.pos << ",";
            trajectoryCreationFile_ << startState.x.vel << ",";
            trajectoryCreationFile_ << startState.x.acc << ",";
        }

        // start state y
        trajectoryCreationFile_ << startState.y.pos << ",";
        trajectoryCreationFile_ << startState.y.vel << ",";
        trajectoryCreationFile_ << startState.y.acc << ",";

        // start state z
        if(!logYonly_) {
            trajectoryCreationFile_ << startState.z.pos << ",";
            trajectoryCreationFile_ << startState.z.vel << ",";
            trajectoryCreationFile_ << startState.z.acc << ",";

            // end state x
            trajectoryCreationFile_ << endState.x.pos << ",";
            trajectoryCreationFile_ << endState.x.vel << ",";
            trajectoryCreationFile_ << endState.x.acc << ",";
        }

        // end state y
        trajectoryCreationFile_ << endState.y.pos << ",";
        trajectoryCreationFile_ << endState.y.vel << ",";
        trajectoryCreationFile_ << endState.y.acc << ",";

        // end state z
        if(!logYonly_) {
            trajectoryCreationFile_ << endState.z.pos << ",";
            trajectoryCreationFile_ << endState.z.vel << ",";
            trajectoryCreationFile_ << endState.z.acc << ",";
        }

        trajectoryCreationFile_ << segment_duration_;

        trajectoryCreationFile_ << std::endl;
    }

    void MyCartesianVelocityController::logCoefficients(){
        coefficientsFile_ << logTime_.sec << ",";
        coefficientsFile_ << logTime_.nsec << ",";
        coefficientsFile_ << "x,";
        coefficientsFile_ << coefs_[0][0] << ",";
        coefficientsFile_ << coefs_[0][1] << ",";
        coefficientsFile_ << coefs_[0][2] << ",";
        coefficientsFile_ << coefs_[0][3] << ",";
        coefficientsFile_ << coefs_[0][4] << ",";
        coefficientsFile_ << coefs_[0][5] << std::endl;

        coefficientsFile_ << logTime_.sec << ",";
        coefficientsFile_ << logTime_.nsec << ",";
        coefficientsFile_ << "y,";
        coefficientsFile_ << coefs_[1][0] << ",";
        coefficientsFile_ << coefs_[1][1] << ",";
        coefficientsFile_ << coefs_[1][2] << ",";
        coefficientsFile_ << coefs_[1][3] << ",";
        coefficientsFile_ << coefs_[1][4] << ",";
        coefficientsFile_ << coefs_[1][5] << std::endl;

        coefficientsFile_ << logTime_.sec << ",";
        coefficientsFile_ << logTime_.nsec << ",";
        coefficientsFile_ << "z,";
        coefficientsFile_ << coefs_[2][0] << ",";
        coefficientsFile_ << coefs_[2][1] << ",";
        coefficientsFile_ << coefs_[2][2] << ",";
        coefficientsFile_ << coefs_[2][3] << ",";
        coefficientsFile_ << coefs_[2][4] << ",";
        coefficientsFile_ << coefs_[2][5] << std::endl;
    }

    void publishState(ros::Time now, const ros::Publisher &pub, const State3 &state){
        util::kinematicState3dStamped msg;
        msg.header.stamp = now;
        msg.state.x.pos  = state.x.pos;
        msg.state.x.vel  = state.x.vel;
        msg.state.x.acc  = state.x.acc;
        msg.state.x.jerk = state.x.jerk;
        msg.state.y.pos  = state.y.pos;
        msg.state.y.vel  = state.y.vel;
        msg.state.y.acc  = state.y.acc;
        msg.state.y.jerk = state.y.jerk;
        msg.state.z.pos  = state.z.pos;
        msg.state.z.vel  = state.z.vel;
        msg.state.z.acc  = state.z.acc;
        msg.state.z.jerk = state.z.jerk;
        pub.publish(msg);
    }

    void MyCartesianVelocityController::update(const ros::Time & /* time */,
                                           const ros::Duration &period) {
        elapsed_time_ += period;
        segment_time_ += period.toSec();

        logTime_ += period;

        //std::cout << period.toSec() << std::endl;

        //std::cerr << "Update\t pos buffer write : " << position_buffer_index_writing_ << "\t read: " << position_buffer_index_reading_ << "\treserve: " << getPositionBufferReserve() << std::endl;

        static bool started = false;
        static int counter = 0;

        // get current pose
        std::array<double, 16> current_pose = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

        // when starting the controller, wait until position buffer is filled with 3 values
        if(!started){
            if(getPositionBufferReserve() >= nominalPositionBufferSize_){
                started = true;
                std::cerr << "Buffer partly filled with " << getPositionBufferReserve() << " entries. Starting-permission granted.\n";
            }
        }

        int max_segments = 0;

        if(started){

            //generalLogFile_ << logTime_.toSec() << "\t" << segment_time_ << "\t" << current_pose[13] << std::endl;

            // if segment_duration_ has passed, calc new trajectory
            if(segment_time_ >= segment_duration_){
                if(getPositionBufferReserve() >= 2){
                    //std::cerr << period.toSec() << "\t" << segment_time_ << std::endl;

                    // at last iteration segment_time was at e.g. 0.009 (if segment_duration_ is 0.01)
                    // last calculation for previous segment has still to be done in order to let new segment start from correct position
                    current_state_.x = evaluatePolynom(coefs_[0], segment_time_);
                    current_state_.y = evaluatePolynom(coefs_[1], segment_time_);
                    current_state_.z = evaluatePolynom(coefs_[2], segment_time_);

                    publishState(logTime_, pub_current_state_, current_state_);
                    logEvaluatedTrajectory();

                    updateTrajectory();
                    generalLogFile_ << logTime_.toSec() << "\t" << "updating trajectory after " << segment_time_ << " s" << std::endl;

                    //std::cerr << "Used one element from position buffer. Remaining: " << getPositionBufferReserve() << std::endl;

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
                else { // if there are not enough values to update trajectory (2 needed), keep last velocity
                    std::cerr << "WARNING: Not enough positions (" << getPositionBufferReserve() << ") to calculate new segment, keep last velocity\n";
                }
            }

            // if within semgent_duration, calc new state
            // can't be "else" to above statement, as it also has to be executed if
            if(segment_time_ <= segment_duration_) {
                // calculat new positions, velocities and accelerations
                current_state_.x = evaluatePolynom(coefs_[0], segment_time_);
                current_state_.y = evaluatePolynom(coefs_[1], segment_time_);
                current_state_.z = evaluatePolynom(coefs_[2], segment_time_);

                publishState(logTime_, pub_current_state_, current_state_);

                logEvaluatedTrajectory();

                generalLogFile_ << logTime_.toSec() << "\t(+" << period.toSec() << ")\t" << "evaluating trajectory" << std::endl;
                logCurrentPosition(current_pose);

                // when debugging
                if (max_segments != 0) {
                    //std::cerr << "[" << current_state_[1][0] << ", " << current_state_[1][1] << ", " << current_state_[1][2] << ", " << elapsed_time_.toSec() << "],\n";
                }

                // compose new velocity
                double vx = current_state_.x.vel;
                double vy = current_state_.y.vel;
                double vz = current_state_.z.vel;

                double wx, wy, wz;
                wx = wy = wz = 0;

                // update command
                current_command_ = {vx, vy, vz, wx, wy, wz};

                commandLogFile_ << logTime_.sec << ", " << logTime_.nsec << ", " << vx << ", " << vy << ", " << vz << std::endl;

                if(exitIfTheoreticalValuesExceedLimits_) {

                    // velocity
                    double vAbs = sqrt(vx * vx + vy * vy + vz * vz);

                    // acceleration
                    double ax = current_state_.x.acc;
                    double ay = current_state_.y.acc;
                    double az = current_state_.z.acc;
                    double aAbs = sqrt(ax * ax + ay * ay + az * az);

                    // jerk
                    double jx = current_state_.x.jerk;
                    double jy = current_state_.y.jerk;
                    double jz = current_state_.z.jerk;
                    double jAbs = sqrt(jx * jx + jy * jy + jz * jz);

                    // boundary values according to https://frankaemika.github.io/docs/control_parameters.html#limit-table
                    const double max_v_trans = 1.7; // m/s
                    const double max_a_trans = 13.0; // m/s²
                    const double max_j_trans = 6500.0; // m/s³

                    bool quit = false;

                    if (jAbs > max_j_trans) {
                        std::cerr << "ERROR: Jerk too high: " << jAbs << std::endl;
                        generalLogFile_ << "ERROR: Jerk too high" << std::endl;
                        quit = true;
                    }

                    if (aAbs > max_a_trans) {
                        std::cerr << "ERROR: Acceleration too high: " << aAbs << std::endl;
                        generalLogFile_ << "ERROR: Acceleration too high" << std::endl;
                        quit = true;
                    }

                    if (vAbs > max_v_trans) {
                        std::cerr << "ERROR: Velocity too high: " << vAbs << std::endl;
                        generalLogFile_ << "ERROR: Velocity too high" << std::endl;
                        quit = true;
                    }

                    if(quit){
                        // print current values of trajectory
                        generalLogFile_ << logTime_.toSec() << "\tv= " << vAbs << "\ta= " << aAbs << "\tj= " << jAbs << std::endl;
                        generalLogFile_ << "Current trajectory coefficients:\n";
                        for(int i = 0; i < 3; ++i){
                            for(int j = 0; j < 6; ++j){
                                generalLogFile_ << coefs_[i][j] << ", ";
                            }
                            generalLogFile_ << std::endl;
                        }
                        //current_command_ = last_command_;
                        exit(-1);
                    }
                }
            }
            else{
                // do nothing, keep current_command_ the same as before
                // -> means that robot should keep current velocity
                //generalLogFile_ << elapsed_time_.toSec() << "\tno new trajectory, keep velocity" << std::endl;
            }

            // pass velocity to robot control
            velocity_cartesian_handle_->setCommand(current_command_);
            //last_command_ = current_command_;
        }

        //rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"
        //pub_error_recovery = node_handle.advertise<franka_msgs::ErrorRecoveryActionGoal>("{}", 20);

        // publish positions for python analytics
        geometry_msgs::PoseStamped msgPose;
        geometry_msgs::Vector3Stamped msgVec;

        msgPose.header.stamp = logTime_;
        msgVec.header.stamp = msgPose.header.stamp;

        // current commanded velocity
        msgVec.vector.x = current_command_[0];
        msgVec.vector.y = current_command_[1];
        msgVec.vector.z = current_command_[2];
        pub_commanded_velocity_.publish(msgVec);

        // current position according to planned trajectory in cartesian space
        msgVec.vector.x = current_state_.x.pos;
        msgVec.vector.y = current_state_.y.pos;
        msgVec.vector.z = current_state_.z.pos;
        pub_current_trajectory_pos_.publish(msgVec);

        // current pos
        msgPose.pose.position.x = current_pose[12];
        msgPose.pose.position.y = current_pose[13];
        msgPose.pose.position.z = current_pose[14];
        pub_current_pose_.publish(msgPose);
    }

    void MyCartesianVelocityController::updateTargetPoseCallback(const geometry_msgs::PoseStamped &msg) {

        // send it back immediately
        geometry_msgs::PoseStamped msgnew = msg;
        msgnew.header.stamp = logTime_;
        pub_current_target_confirmation_.publish(msgnew);

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

        // index of next positions
        int i1 = (position_buffer_index_reading_ + 1) % position_buffer_length_; // next position
        int i2 = (position_buffer_index_reading_ + 2) % position_buffer_length_; // second next position

        // startState equals current state, except position might be taken from robot end-effector position
        State3 startState = current_state_;

        if(useActualRobotPosition_){
            // get current pose
            std::array<double, 16> current_pose = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

            startState.x.pos = current_pose[12];
            startState.y.pos = current_pose[13];
            startState.z.pos = current_pose[14];
        }

        State3 endState;

        endState.x.pos = position_buffer_[i1][0];
        endState.y.pos = position_buffer_[i1][1];
        endState.z.pos = position_buffer_[i1][2];

        // calculate desired velocity for end of (first) segment as mean velocity of next two segments
        endState.x.vel = (position_buffer_[i2][0] - startState.x.pos)/(segment_duration_*2);
        endState.y.vel = (position_buffer_[i2][1] - startState.y.pos)/(segment_duration_*2);
        endState.z.vel = (position_buffer_[i2][2] - startState.z.pos)/(segment_duration_*2);

        // calculate desired acceleration for end of (first) segment as mean acceleration of next two segments
        endState.x.acc = (endState.x.vel - startState.x.vel)/segment_duration_;
        endState.y.acc = (endState.y.vel - startState.y.vel)/segment_duration_;
        endState.z.acc = (endState.z.vel - startState.z.vel)/segment_duration_;

        // calculate polynom coefficients
        // TODO: using current velocity and acceleration from current_state_ vector is not 100% correct, as they are the vel and acc from last step
        coefs_[0] = calcCoefs(startState.x, endState.x, segment_duration_);
        coefs_[1] = calcCoefs(startState.y, endState.y, segment_duration_);
        coefs_[2] = calcCoefs(startState.z, endState.z, segment_duration_);

        logTrajectoryCreation(startState, endState);
        logCoefficients();

        // for next segment
        position_buffer_index_reading_ = (position_buffer_index_reading_ + 1) % position_buffer_length_;
    }

    void MyCartesianVelocityController::stopping(const ros::Time & /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.

        generalLogFile_.close();
        commandLogFile_.close();
        evaluatedTrajectoryFile_.close();
        currentPositionFile_.close();
        trajectoryCreationFile_.close();
        coefficientsFile_.close();
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyCartesianVelocityController,
        controller_interface::ControllerBase
)
