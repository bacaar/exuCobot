// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/my_cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>

#include <thread>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

//std::vector<std::string> s_openLogFiles;

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
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: Polynomial degree for interpolation not implemented" << std::endl;
            #else
            std::cerr << "ERROR: Polynomial degree for interpolation not implemented" << std::endl;
            #endif
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
        current_target_ = std::vector<double>(4, 0);    // (x, y, z, dt)
        position_buffer_ = std::vector<Command>(position_buffer_length_, {0, 0, 0, 0});
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

        last_command_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize variable, assume that robot is standing still

        logTime_ = ros::Time(0);
        segment_time_ = ros::Duration(0);

        elapsed_time_ = ros::Duration(0.0);

        #if ENABLE_LOGGING

        std::cout << "This thread's id: " << std::this_thread::get_id() << std::endl;

        textLogger_ = std::make_shared<TextLogger>("textLog.log", LogLevel::Debug, false, true);
        evalTrajLogger_ = std::make_shared<CsvLogger>("evalTrajLog.csv");

        textLogger_->log("This is a info msg", LogLevel::Info);
        evalTrajLogger_->log("rt,t,px,vx,ax,jx,py,vy,ay,jy,pz,vz,az,jz");

        logThreader_.addLogger(textLogger_);
        logThreader_.addLogger(evalTrajLogger_);

        generalLogFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/general.log", std::ios::out);
        targetLogFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/targetVC.csv", std::ios::out);
        commandLogFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/commands.csv", std::ios::out);
        evaluatedTrajectoryFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/evaluatedTrajectory.csv", std::ios::out);
        currentPositionFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/currentPositionVC.csv", std::ios::out);
        trajectoryCreationFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/trajectoryCreation.csv", std::ios::out);
        coefficientsFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/coefficientsCppController.csv", std::ios::out);
        trajectoryCreationFile2_.open("/home/robocup/catkinAaron/src/exuCobot/log/trajectoryCreation2.csv", std::ios::out);

        if(!evaluatedTrajectoryFile_.is_open()) { std::cerr << "WARNING: Could not create open evaluated trajectory log file!" << std::endl; }
        else {
            if(logYonly_) {
                evaluatedTrajectoryFile_ << "rt,t,py,vy,ay,jy\n";
            }
            else{
                evaluatedTrajectoryFile_ << "rt,t,px,vx,ax,jx,py,vy,ay,jy,pz,vz,az,jz\n";
            }
        }

        if(!targetLogFile_.is_open()) { std::cerr << "WARNING: Could not create open target log file!\n"; }
        else {
            targetLogFile_ << "rt,t,px,py,pz,dt\n";
        }

        if(!commandLogFile_.is_open()) { std::cerr << "WARNING: Could not create open evaluated trajectory log file!\n"; }
        else {
            if(logYonly_) {
                commandLogFile_ << "rt,t,vy\n";
            }
            else{
                commandLogFile_ << "rt,t,vx,vy,vz\n";
            }
        }

        if(!currentPositionFile_.is_open()) { std::cerr << "WARNING: Could not create open current position log file!\n"; }
        else {
            currentPositionFile_ << "rt,t,px,py,pz,q0,q1,q2,q3,q4,q5,q6\n";
        }

        if(!trajectoryCreationFile_.is_open()) { std::cerr << "WARNING: Could not create open trajectory creation log file!\n"; }
        else {
            // cpx = current position x, cvx = current velocity x, etc...
            // npx = next position x, etc...
            // dt = segment_timeobserving position, velocity acceleration and jerk of robot in cartesian and space6
            if(logYonly_) {
                trajectoryCreationFile_ << "rt,t,cpy,cvy,cay,npy,nvy,nay,dt\n";
            }
            else{
                trajectoryCreationFile_ << "rt,t,cpx,cvx,cax,cpy,cvy,cay,cpz,cvz,caz,npx,nvx,nax,npy,nvy,nay,npz,nvz,naz,dt\n";
            }
        }

        if(!coefficientsFile_.is_open()) { std::cerr << "WARNING: Could not create open trajectory creation log file!\n"; }
        else {
                coefficientsFile_ << "rt,t,coord,A,B,C,D,E,F,dt\n";
        }

        if(!trajectoryCreationFile2_.is_open()) { std::cerr << "WARNING: Could not create open trajectory creation log file!\n"; }
        else {
            trajectoryCreationFile2_ << "rt,t,cpy,npy,cvy,nvy,cay,nay,dpy,dvy,day,dt\n";
        }
        #endif
    }

    const int MyCartesianVelocityController::getPositionBufferReserve(){

        if (position_buffer_index_writing_ > position_buffer_index_reading_){
            return position_buffer_index_writing_ - position_buffer_index_reading_ - 1;
        }
        else if (position_buffer_index_writing_ < position_buffer_index_reading_){
            return position_buffer_index_writing_ + position_buffer_length_ - position_buffer_index_reading_- 1;
        }
        else{

            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: Writing index has caught reading index" << std::endl;
            #else
            std::cerr << "ERROR: Writing index has caught reading index" << std::endl;
            #endif
            exit(-1);
        }
    }

    std::vector<double> MyCartesianVelocityController::calcCoefs(State startState, State endState, double T){

        assert(T > 0);

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

    State MyCartesianVelocityController::evaluatePolynomial(std::vector<double> &coef, double t){

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

    // function to format ros::Time as s.ns
    std::string getRosTimeString(ros::Time time){

        std::string sec = std::to_string(time.sec);
        std::string nsec = std::to_string(time.nsec);

        // nsec should have 9 digits, else fill front up with zeros
        int nZeros = 9 - nsec.length();
        std::string zeros = "";
        for(int i = 0; i < nZeros; ++i){
            zeros += "0";
        }

        std::string res = sec + "." + zeros + nsec;
        return res;
    }

    #if ENABLE_LOGGING
    void MyCartesianVelocityController::logEvaluatedTrajectory(){

        evaluatedTrajectoryFile_ << rosTimeString_ << "," << logTimeString_ << ",";

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

        std::string msg = rosTimeString_ + "," + logTimeString_ + ","
                + std::to_string(current_state_.x.pos) + ","
                + std::to_string(current_state_.x.vel) + ","
                + std::to_string(current_state_.x.acc) + ","
                + std::to_string(current_state_.x.jerk) + ","
                + std::to_string(current_state_.y.pos) + ","
                + std::to_string(current_state_.y.vel) + ","
                + std::to_string(current_state_.y.acc) + ","
                + std::to_string(current_state_.y.jerk) + ","
                + std::to_string(current_state_.z.pos) + ","
                + std::to_string(current_state_.z.vel) + ","
                + std::to_string(current_state_.z.acc) + ","
                + std::to_string(current_state_.z.jerk);

        evalTrajLogger_->log(msg);
    }

    void MyCartesianVelocityController::logCurrentPosition(const std::array<double, 16> &current_robot_state, const std::array< double, 7 > &current_joint_positions) {
        currentPositionFile_ << rosTimeString_ << "," << logTimeString_ << ",";
        currentPositionFile_ << current_robot_state[12] << ",";
        currentPositionFile_ << current_robot_state[13] << ",";
        currentPositionFile_ << current_robot_state[14] << ",";
        currentPositionFile_ << current_joint_positions[0] << ",";
        currentPositionFile_ << current_joint_positions[1] << ",";
        currentPositionFile_ << current_joint_positions[2] << ",";
        currentPositionFile_ << current_joint_positions[3] << ",";
        currentPositionFile_ << current_joint_positions[4] << ",";
        currentPositionFile_ << current_joint_positions[5] << ",";
        currentPositionFile_ << current_joint_positions[6];
        currentPositionFile_ << std::endl;
    }

    void MyCartesianVelocityController::logTrajectoryCreation(const State3 &startState, const State3 &endState){

        trajectoryCreationFile_ << rosTimeString_ << "," << logTimeString_ << ",";

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

        trajectoryCreationFile_ << segment_duration_.toSec();

        trajectoryCreationFile_ << std::endl;
    }

    void MyCartesianVelocityController::logCoefficients(){

        char coords[] = {'x', 'y', 'z'};

        for(int i = 0; i < 3; ++i) {    // for each of the three coordinates x y z
            coefficientsFile_ << rosTimeString_ << "," << logTimeString_ << ",";
            coefficientsFile_ << coords[i] << ",";
            for(int j = 0; j < 6; ++j) {    // for every coefficient
                coefficientsFile_ << coefs_[i][j];
                if (j < 5) coefficientsFile_ << ",";
                else coefficientsFile_ << std::endl;
            }
        }
    }
    #endif

    void MyCartesianVelocityController::publishState(ros::Time now, const State3 &state){
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
        pub_current_state_.publish(msg);
    }

    void MyCartesianVelocityController::update(const ros::Time &time,
                                           const ros::Duration &period) {

        elapsed_time_ += period;
        segment_time_ += period;

        #if ENABLE_LOGGING
        static ros::Time lastRosTime = ros::Time(0);
        static ros::Time lastLogTime = ros::Time(0);

        logTime_ += period;
        logTimeString_ = getRosTimeString(logTime_);
        rosTimeString_ = getRosTimeString(time);
        std::string timeStr = logTimeString_ + ", " + rosTimeString_;

        generalLogFile_ << "[" << rosTimeString_ << "] (+ " << (time - lastRosTime).toSec();
        generalLogFile_ << ")\t[" << logTimeString_ << "] (+ " << (logTime_ - lastLogTime).toSec();
        generalLogFile_ << ") new step after " << period.toSec() << " s" << std::endl;

        std::string msg = "new step after " + std::to_string(period.toSec()) + " s";
        textLogger_->log(msg, LogLevel::Debug, timeStr);

        lastRosTime = time;
        lastLogTime = logTime_;
        #endif

        //std::cout << period.toSec() << std::endl;

        //std::cerr << "Update\t pos buffer write : " << position_buffer_index_writing_ << "\t read: " << position_buffer_index_reading_ << "\treserve: " << getPositionBufferReserve() << std::endl;

        static bool started = false;
        static int counter = 0;

        // get current pose
        std::array<double, 16> current_robot_state= velocity_cartesian_handle_->getRobotState().O_T_EE_d;

        // when starting the controller, wait until position buffer is filled with 3 values
        if(!started){
            if(getPositionBufferReserve() >= nominalPositionBufferSize_){
                started = true;
                std::cerr << "Buffer partly filled with " << getPositionBufferReserve() << " entries. Starting-permission granted." << std::endl;
            }
        }

        if(started){

            //generalLogFile_ << logTime_.toSec() << "\t" << segment_time_ << "\t" << current_robot_state[13] << std::endl;

            // if segment_duration_ has passed, calc new trajectory
            if(segment_time_ >= segment_duration_){
                #if ENABLE_LOGGING
                generalLogFile_ << "new Trajectory needed" << std::endl;
                #endif

                if(getPositionBufferReserve() >= 1){

                    //overdueTime_ += segment_time_ - segment_duration_;
                    overdueTime_ = segment_time_ - segment_duration_;

                    // when getting in here the first time, overdue time quite big (several seconds), depending on how long
                    // it takes to start exudyn. So ignore first time
                    static bool firstTime = true;
                    if(firstTime){
                        overdueTime_ = ros::Duration(0);
                        firstTime = false;
                    }

                    #if ENABLE_LOGGING
                    if (overdueTime_ < ros::Duration(0)) {
                        generalLogFile_ << "ERROR: overdue Time < 0" << std::endl;
                    }
                    else {
                        generalLogFile_ << "Overdue time: " << overdueTime_ << std::endl;
                    }

                    generalLogFile_ << "updating trajectory after " << segment_time_ << " s" << std::endl;
                    trajectoryCreationFile2_ << rosTimeString_ << "," << logTimeString_ << ",";
                    #endif
                    updateTrajectory();

                    // reset segment_time_ as new one starts now
                    //segment_time_ = ros::Duration(0);
                    segment_time_ = overdueTime_;
                    #if ENABLE_LOGGING
                    //generalLogFile_ << "setting segment_time_ to 0" << std::endl;
                    generalLogFile_ << "setting segment_time_ to " << segment_time_ << std::endl;
                    #endif
                }
                else { // if there is no further entry in position_buffer_, keep last velocity
                    #if ENABLE_LOGGING
                    std::cerr << "WARNING: No further entry to calculate new segment available, keep last velocity" << std::endl;
                    generalLogFile_ << "WARNING: No further entry to calculate new segment available, keep last velocity" << std::endl;
                    #endif

                    if(exitIfPositionBufferEmpty_) {
                        #if ENABLE_LOGGING
                        generalLogFile_ << "ERROR: Position buffer empty" << std::endl;
                        #else
                        std::cout << "ERROR: Position buffer empty" << std::endl;
                        #endif
                        exit(-1);
                    }

                    // just update current state
                    // update position; velocity remains the same and therefor acceleration is 0
                    current_state_.x = {current_robot_state[12], current_state_.x.vel, 0};
                    current_state_.y = {current_robot_state[13], current_state_.y.vel, 0};
                    current_state_.z = {current_robot_state[14], current_state_.z.vel, 0};
                }
            }

            // if within segment_duration, calc new state
            // can't be "else" to above statement, as it also has to be executed if trajectory has just been updated
            if(segment_time_ <= segment_duration_) {
                // calculate new positions, velocities and accelerations
                #if ENABLE_LOGGING
                generalLogFile_ << "evaluating trajectory" << std::endl;
                #endif
                current_state_.x = evaluatePolynomial(coefs_[0], segment_time_.toSec());
                current_state_.y = evaluatePolynomial(coefs_[1], segment_time_.toSec());
                current_state_.z = evaluatePolynomial(coefs_[2], segment_time_.toSec());

                //publishState(logTime_, current_state_);

                // get current pose
                std::array<double, 7> current_joint_positions= velocity_cartesian_handle_->getRobotState().q;

                #if ENABLE_LOGGING
                logEvaluatedTrajectory();
                logCurrentPosition(current_robot_state, current_joint_positions);
                #endif

                // compose new velocity
                double vx = current_state_.x.vel;
                double vy = current_state_.y.vel;
                double vz = current_state_.z.vel;

                double wx, wy, wz;
                wx = wy = wz = 0;

                // update command
                current_command_ = {vx, vy, vz, wx, wy, wz};

                if(exitIfTheoreticalValuesExceedLimits_) {

                    // check if v, a and j are within absolute limits, else reduce them (especially velocity)

                    // acceleration
                    double ax = current_state_.x.acc;
                    double ay = current_state_.y.acc;
                    double az = current_state_.z.acc;

                    // jerk
                    double jx = current_state_.x.jerk;
                    double jy = current_state_.y.jerk;
                    double jz = current_state_.z.jerk;
                    double jAbs = sqrt(jx * jx + jy * jy + jz * jz);

                    double factorJ = jAbs / max_j_trans_;

                    if(factorJ > 1.0){
                        #if ENABLE_LOGGING
                        generalLogFile_ << "Jerk by factor " << factorJ << " too high. Adapting" << std::endl;
                        #endif

                        // update jerk
                        jx /= factorJ;
                        jy /= factorJ;
                        jz /= factorJ;

                        // calculate new velocity according to v = v0 + 1/2 * j1 * t²
                        last_command_[0] += 0.5 * jx * pow(period.toSec(), 2);
                        last_command_[1] += 0.5 * jy * pow(period.toSec(), 2);
                        last_command_[2] += 0.5 * jz * pow(period.toSec(), 2);

                        // calculate new acceleration according to a = a0 + j1 * t
                        ax += jx * period.toSec();
                        ay += jy * period.toSec();
                        az += jz * period.toSec();
                    }

                    double aAbs = sqrt(ax * ax + ay * ay + az * az);
                    double factorA = aAbs / max_a_trans_;

                    if(factorA > 1.0){
                        #if ENABLE_LOGGING
                        generalLogFile_ << "Acceleration by factor " << factorA << " too high. Adapting" << std::endl;
                        #endif

                        // update acceleration
                        ax /= factorA;
                        ay /= factorA;
                        az /= factorA;

                        // calculate new velocity according to v = v0 + a1 * t
                        last_command_[0] += ax / factorA * period.toSec();
                        last_command_[1] += ay / factorA * period.toSec();
                        last_command_[2] += az / factorA * period.toSec();
                    }

                    double vAbs = sqrt(vx * vx + vy * vy + vz * vz);
                    double factorV = vAbs / max_v_trans_;

                    if(factorV > 1.0){
                        #if ENABLE_LOGGING
                        generalLogFile_ << "Velocity by factor " << factorV << " too high. Adapting" << std::endl;
                        #endif

                        // update velocity (according to change since last command)
                        vx = last_command_[0] + (vx - last_command_[0]) / factorV;
                        vy = last_command_[1] + (vy - last_command_[1]) / factorV;
                        vz = last_command_[2] + (vz - last_command_[2]) / factorV;
                    }

                    jAbs = sqrt(jx * jx + jy * jy + jz * jz);
                    aAbs = sqrt(ax * ax + ay * ay + az * az);
                    vAbs = sqrt(vx * vx + vy * vy + vz * vz);
                    bool quit = false;

                    // add small constant because of numerical inaccuracy
                    if (jAbs > max_j_trans_ + 0.0001) {
                        #if ENABLE_LOGGING
                        std::cerr << "ERROR: Jerk too high: " << jAbs << std::endl;
                        generalLogFile_ << "ERROR: Jerk too high" << std::endl;
                        #endif
                        quit = true;
                    }

                    if (aAbs > max_a_trans_ + 0.0001) {
                        #if ENABLE_LOGGING
                        std::cerr << "ERROR: Acceleration too high: " << aAbs << std::endl;
                        generalLogFile_ << "ERROR: Acceleration too high" << std::endl;
                        #endif
                        quit = true;
                    }

                    if (vAbs > max_v_trans_ + 0.0001) {
                        #if ENABLE_LOGGING
                        std::cerr << "ERROR: Velocity too high: " << vAbs << std::endl;
                        generalLogFile_ << "ERROR: Velocity too high" << std::endl;
                        #endif
                        quit = true;
                    }

                    if(quit){
                        // print current values of trajectory
                        #if ENABLE_LOGGING
                        generalLogFile_ << logTime_.toSec() << "\tv= " << vAbs << "\ta= " << aAbs << "\tj= " << jAbs << std::endl;
                        generalLogFile_ << "Current trajectory coefficients:" << std::endl;
                        for(int i = 0; i < 3; ++i){
                            for(int j = 0; j < 6; ++j){
                                generalLogFile_ << coefs_[i][j] << ", ";
                            }
                            generalLogFile_ << std::endl;
                        }
                        generalLogFile_ << "ERROR: Movement discontinuity detected" << std::endl;
                        #endif
                        //current_command_ = last_command_;
                        exit(-1);
                    }

                    current_command_ = {vx, vy, vz, wx, wy, wz};
                }
            }

            // check if new command is possible
            // max velocity change is max_acc * dt
            double max_dv = max_a_trans_ * 0.001;
            for(int i = 0; i < 3; ++i){ // check for all three axes

                // velocity change
                int dv = current_command_[i]-last_command_[i];

                if(dv >= max_dv){
                    //current_command_[i] = last_command_[i];     // keep last velocity
                    //std::cerr << "WARNING: Keeping velocity for axis " << i << std::endl;
                    #if ENABLE_LOGGING
                    std::cerr << "Last v: " << last_command_[i] << "\t next v: " << current_command_[i];
                    std::cerr << "\tdv: " << dv;
                    std::cerr << "\texceeds limit dv of " << max_dv;
                    #endif

                    // velocity increasing or decreasing
                    int sign = current_command_[i] > last_command_[i] ? 1 : -1;

                    // applying max velocity change but not more
                    current_command_[i] = last_command_[i] + max_dv * sign;

                    #if ENABLE_LOGGING
                    std::cerr << "\t-> next v: " << current_command_[i] << std::endl;
                    #endif
                }
            }

            #if ENABLE_LOGGING
            // pass velocity to robot control and log it
            commandLogFile_ << rosTimeString_ << "," << logTimeString_ << ", " << current_command_[0] << ", " << current_command_[1] << ", " << current_command_[2] << std::endl;
            #endif

            // check if one of commanded velocities is NaN. Can't reproduce error but once I got a "FrankaHW::controlCallback: Got NaN command!" fatal error
            for(int i = 0; i < 6; ++i){
                if(isnan(current_command_[i])){
                    #if ENABLE_LOGGING
                    generalLogFile_ << "ERROR: Command [" << i << "] is NaN" << std::endl;
                    generalLogFile_ << "Segment time: " << segment_time_ << "\tSegment duration: " << segment_duration_ << "\toverdue time: " << overdueTime_ << std::endl;
                    #endif
                    exit(-1);
                }
            }

            velocity_cartesian_handle_->setCommand(current_command_);
            last_command_ = current_command_;
        }

        double updateTime = (ros::Time::now() - time).toSec();
        double updateTimePrintThreshold = 1e-4;

        #if ENABLE_LOGGING
        if(updateTime > updateTimePrintThreshold) { // update call on average takes 3e-5 - 5e-5 seconds. just print it when significantly above
            generalLogFile_ << "Update call took over " << updateTimePrintThreshold << " s ( " << updateTime << " s)" << std::endl;
        }
        #endif

        //rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"
        //pub_error_recovery = node_handle.advertise<franka_msgs::ErrorRecoveryActionGoal>("{}", 20);

        /*
        // publish positions for python analytics
        geometry_msgs::Vector3Stamped msgVec;
        msgVec.header.stamp = logTime_;

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
        */

        // current pos
        geometry_msgs::PoseStamped msgPose;
        msgPose.header.stamp = logTime_;
        msgPose.pose.position.x = current_robot_state[12];
        msgPose.pose.position.y = current_robot_state[13];
        msgPose.pose.position.z = current_robot_state[14];
        pub_current_pose_.publish(msgPose);
    }

    void MyCartesianVelocityController::updateTargetPoseCallback(const util::segmentCommand &msg) {

        // send it back immediately
        /*geometry_msgs::PoseStamped msgnew;
        msgnew.pose.position.x = msg.x;
        msgnew.pose.position.y = msg.y;
        msgnew.pose.position.z = msg.z;
        msgnew.header.stamp = logTime_;
        pub_current_target_confirmation_.publish(msgnew);*/

        if(position_buffer_index_writing_ == position_buffer_index_reading_){
            std::cerr << "ERROR: Position buffer full" << std::endl;
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: Position buffer full" << std::endl;
            #endif
            exit(-1);
        }

        State3 state;
        state.x = {msg.x.pos, msg.x.vel, msg.x.acc, 0.0};
        state.y = {msg.y.pos, msg.y.vel, msg.y.acc, 0.0};
        state.z = {msg.z.pos, msg.z.vel, msg.z.acc, 0.0};

        position_buffer_[position_buffer_index_writing_] = {state, msg.dt};

        if(msg.dt <= 0){
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: dt of new segment must be >0" << std::endl;
            #else
            std::cerr << "ERROR: dt of new segment must be >0" << std::endl;
            #endif
            exit(-1);
        }

        int old = position_buffer_index_writing_;
        position_buffer_index_writing_ = (position_buffer_index_writing_ + 1) % position_buffer_length_;

        // for analytics
        current_target_[0] = msg.x.pos;
        current_target_[1] = msg.y.pos;
        current_target_[2] = msg.z.pos;
        current_target_[3] = msg.dt;

        #if ENABLE_LOGGING
        targetLogFile_ << rosTimeString_ << "," << logTimeString_ << ",";
        targetLogFile_ << msg.x << "," << msg.y << "," << msg.z << "," << msg.dt << std::endl;
        #endif
    }

    double cartesianDistance(std::vector<double> s1, std::vector<double> s2){
        double dx = s2[0] - s1[0];
        double dy = s2[1] - s1[1];
        double dz = s2[2] - s1[2];
        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    void roundState(State &state, int precission){
        double multiplier = pow(10, precission);

        state.pos = round(state.pos * multiplier)/multiplier;
        state.vel = round(state.vel * multiplier)/multiplier;
        state.acc = round(state.acc * multiplier)/multiplier;
    }

    void roundState3(State3 &state, int precission){
        roundState(state.x, precission);
        roundState(state.y, precission);
        roundState(state.z, precission);
    }

    void MyCartesianVelocityController::updateTrajectory() {

        // at last iteration segment_time was at e.g. 0.009 (if segment_duration_ is 0.01)
        // last calculation for previous segment has still to be done in order to let new segment start from correct position
        current_state_.x = evaluatePolynomial(coefs_[0], segment_duration_.toSec());
        current_state_.y = evaluatePolynomial(coefs_[1], segment_duration_.toSec());
        current_state_.z = evaluatePolynomial(coefs_[2], segment_duration_.toSec());

        #if ENABLE_LOGGING
        //publishState(logTime_, current_state_);
        logEvaluatedTrajectory();
        generalLogFile_ << "evaluating trajectory" << std::endl;
        #endif

        // get current robot state
        std::array<double, 16> current_robot_state = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

        // when receiving first time a trajectory, coefficients are still all on zero at this point (and therefor current_state, too)
        // therefore use current_robot_state as current_state
        if (current_state_.x.pos == 0 && current_state_.y.pos == 0 && current_state_.z.pos == 0) {
            current_state_.x.pos = current_robot_state[12];
            current_state_.y.pos = current_robot_state[13];
            current_state_.z.pos = current_robot_state[14];
        }

        // startState equals current state, except .pos might be taken from robot end-effector position
        State3 startState = current_state_;

        double distance = cartesianDistance({current_state_.x.pos, current_state_.y.pos, current_state_.z.pos},
                                            {current_robot_state[12], current_robot_state[13], current_robot_state[14]});

        // distance must not be bigger than maximal distance which can be covered in one step (0.001s) with max velocity
        if (distance >= max_v_trans_ * 0.001) {
            std::cerr << "[" << rosTimeString_ << "] ERROR: Robot and Controller not in sync! Cartesian distance: " << distance << " m" << std::endl;
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: Robot and Controller not in sync! Cartesian distance: " << distance << " m" << std::endl;
            #endif
            exit(-1);
        }

        // index of next positions
        int i1 = (position_buffer_index_reading_ + 1) % position_buffer_length_; // next position

        if(position_buffer_[i1].dt <= 0){
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: Desired segment-time must be >0" << std::endl;
            #else
            std::cout << "ERROR: Desired segment-time must be >0" << std::endl;
            #endif
            exit(-1);
        }

        // calc new segment_duration_
        /*
        // substract overdueRecoverage from segment_duration to recover overdue time
        ros::Duration maxOverdueRecoverage = ros::Duration(position_buffer_[i1].dt * maxOverdueRecoverPercentage_);
        ros::Duration overdueRecoverage = std::min(maxOverdueRecoverage, overdueTime_);

        if(overdueRecoverage > ros::Duration(0)) {
            #if ENABLE_LOGGING
            generalLogFile_ << "recovering overdue by " << overdueRecoverage << " s" << std::endl;
            #endif
            segment_duration_ = ros::Duration(position_buffer_[i1].dt) - overdueRecoverage;
            overdueTime_ -= overdueRecoverage;
            #if ENABLE_LOGGING
            generalLogFile_ << "remaining overdue time: " << overdueTime_.toSec() << std::endl;
            #endif
        }
        else{
            segment_duration_ = ros::Duration(position_buffer_[i1].dt);
        }
        */
        segment_duration_ = ros::Duration(position_buffer_[i1].dt);

        #if ENABLE_LOGGING
        generalLogFile_ << "new segment_duration_ is " << segment_duration_.toSec() << " s" << std::endl;
        #endif

        // calculate trajectory endstate
        State3 endState;

        // get endState._.pos from position buffer
        for(int c = 0; c < 3; ++c){     // for coordinates c = x, y, z
            endState[c].pos = position_buffer_[i1][c].pos;
            endState[c].vel = position_buffer_[i1][c].vel;
            endState[c].acc = position_buffer_[i1][c].acc;
        }

        //roundState3(startState, 6);
        //roundState3(endState, 6);
        #if ENABLE_LOGGING
        trajectoryCreationFile2_ << startState.y.pos << ",\t" << endState.y.pos << ",\t";   // pos
        trajectoryCreationFile2_ << startState.y.vel << ",\t" << endState.y.vel << ",\t";   // vel
        trajectoryCreationFile2_ << startState.y.acc << ",\t" << endState.y.acc << ",\t";   // acc
        trajectoryCreationFile2_ << (endState.y.pos - startState.y.pos) << ",\t";           // dp1
        trajectoryCreationFile2_ << (endState.y.vel - startState.y.vel) << ",\t";           // dv
        trajectoryCreationFile2_ << (endState.y.acc - startState.y.acc) << ",\t";           // da
        trajectoryCreationFile2_ << segment_duration_.toSec() << std::endl;
        #endif

        // calculate polynom coefficients
        coefs_[0] = calcCoefs(startState.x, endState.x, segment_duration_.toSec());
        coefs_[1] = calcCoefs(startState.y, endState.y, segment_duration_.toSec());
        coefs_[2] = calcCoefs(startState.z, endState.z, segment_duration_.toSec());

        #if ENABLE_LOGGING
        logTrajectoryCreation(startState, endState);
        logCoefficients();
        #endif

        // for next segment
        position_buffer_index_reading_ = (position_buffer_index_reading_ + 1) % position_buffer_length_;
    }

    void MyCartesianVelocityController::stopping(const ros::Time & /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.

        #if ENABLE_LOGGING
        generalLogFile_.close();
        targetLogFile_.close();
        commandLogFile_.close();
        evaluatedTrajectoryFile_.close();
        currentPositionFile_.close();
        trajectoryCreationFile_.close();
        coefficientsFile_.close();
        trajectoryCreationFile2_.close();
        #endif
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyCartesianVelocityController,
        controller_interface::ControllerBase
)
