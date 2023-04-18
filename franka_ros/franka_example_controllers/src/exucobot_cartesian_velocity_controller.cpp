// Aaron Bacher, March 2022 - March 2023
// aaronbacher@gmx.de
//
// based on example code from Franka Emika
// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka_example_controllers/exucobot_cartesian_velocity_controller.h>

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

namespace franka_example_controllers {

    bool ExuCobotCartesianVelocityController::init(hardware_interface::RobotHW *robot_hardware,
                                                   ros::NodeHandle &node_handle) {
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("ExuCobotCartesianVelocityController: Could not get parameter arm_id");
            return false;
        }

        velocityCartesianInterface_ = robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
        if (velocityCartesianInterface_ == nullptr) {
            ROS_ERROR(
                    "ExuCobotCartesianVelocityController: Could not get Cartesian velocity interface from "
                    "hardware");
            return false;
        }

        try {
            velocityCartesianHandle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
                    velocityCartesianInterface_->getHandle(arm_id + "_robot"));
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR_STREAM(
                    "ExuCobotCartesianVelocityController: Exception getting Cartesian handle: " << e.what());
            return false;
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("ExuCobotCartesianVelocityController: Could not get state interface from hardware");
            return false;
        }

        try {
          auto state_handle = state_interface->getHandle(arm_id + "_robot");
        } catch (const hardware_interface::HardwareInterfaceException& e) {
          ROS_ERROR_STREAM(
              "ExuCobotCartesianVelocityController: Exception getting state handle: " << e.what());
          return false;
        }

        // set callback method for updating reference pose
        subDesiredPose_ = node_handle.subscribe("referencePose", 20,
                                                  &ExuCobotCartesianVelocityController::updateReferencePoseCallback, this,
                                                  ros::TransportHints().reliable().tcpNoDelay());

        if(polynomialDegree_ != 3 && polynomialDegree_ != 5){
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: Polynomial degree for interpolation not implemented" << std::endl;
            #endif
            std::cerr << "ERROR: Polynomial degree for interpolation not implemented" << std::endl;
            exit(-1);
        }

        // create publisher for returning reference pose
        pubCurrentReferenceConfirmation_ = node_handle.advertise<geometry_msgs::PoseStamped>("currentReference", 20);

        // create publisher for current command
        pubCommandedVelocity_ = node_handle.advertise<geometry_msgs::Vector3Stamped>("commandedVelocity", 20);

        // create publisher for current pose
        pubCurrentTrajectoryPos_ = node_handle.advertise<geometry_msgs::Vector3Stamped>("evaluatedTrajectory", 20);

        // create publisher for current pose
        pubCurrentPose_ = node_handle.advertise<geometry_msgs::PoseStamped>("currentPose", 20);

        // create publisher for current state
        pubCurrentState_ = node_handle.advertise<util::posVelAccJerk3dStamped>("currentState", 20);

        // initialize variables
        currentReference__ = std::vector<double>(4, 0);    // (x, y, z, dt)
        positionBuffer_ = std::vector<Command>(positionBufferLength_, {0, 0, 0, 0});
        positionBufferReadingIndex_ = 0;
        positionBufferWritingIndex_ = 1;
        coefs_ = std::vector<std::vector<double>>(3, std::vector<double>(6, 0));

        std::cout << "INFO: Starting velocity Controller with interpolation polynomial degree " << polynomialDegree_;
        std::cout << " and nominal position buffer size " << minimalPositionBufferSize_ << std::endl;

        return true;
    }

    void ExuCobotCartesianVelocityController::starting(const ros::Time & /* time */) {
        std::array<double, 16> initial_pose = velocityCartesianHandle_->getRobotState().O_T_EE_d;

        // set next positions on current positions to stay here (NOT ZERO!!!)
        // TODO can I let this on zero because of my brilliant ring buffer logic?

        // set position entries of currentState_ vector to current positions
        currentState_.x.pos = initial_pose[12];
        currentState_.y.pos = initial_pose[13];
        currentState_.z.pos = initial_pose[14];

        lastCommand_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initialize variable, assume that robot is standing still

        // initialize timing variables
        logTime_ = ros::Time(0);
        segmentTime_ = ros::Duration(0);
        elapsedTime_ = ros::Duration(0.0);

        #if ENABLE_LOGGING
        // do some logging and initialize loggers / open log files and write csv header
        std::cout << "This thread's id: " << std::this_thread::get_id() << std::endl;

        textLogger_ = std::make_shared<TextLogger>("/home/robocup/catkinAaron/src/exuCobot/log/textLog.log", LogLevel::Debug, true, false, true);
        evalTrajLogger_ = std::make_shared<CsvLogger>("evalTrajLog.csv");

        textLogger_->log("This is a info msg", LogLevel::Info);
        evalTrajLogger_->log("rt,t,px,vx,ax,jx,py,vy,ay,jy,pz,vz,az,jz");

        logThreader_.addLogger(textLogger_);
        logThreader_.addLogger(evalTrajLogger_);

        generalLogFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/general.log", std::ios::out);
        referenceLogFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/referenceVC.csv", std::ios::out);
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

        if(!referenceLogFile_.is_open()) { std::cerr << "WARNING: Could not create open reference log file!\n"; }
        else {
            referenceLogFile_ << "rt,t,px,py,pz,dt\n";
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

    const int ExuCobotCartesianVelocityController::getPositionBufferReserve(){

        // depending on wheter writing or reading index is smaller/bigger, reserve has to be calculated in a specific way
        if (positionBufferWritingIndex_ > positionBufferReadingIndex_){
            return positionBufferWritingIndex_ - positionBufferReadingIndex_ - 1;
        }
        else if (positionBufferWritingIndex_ < positionBufferReadingIndex_){
            return positionBufferWritingIndex_ + positionBufferLength_ - positionBufferReadingIndex_- 1;
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

    std::vector<double> ExuCobotCartesianVelocityController::calcCoefs(SamplingPoint startState, SamplingPoint endState, double T){

        assert(T > 0);

        // T^2 and T^3 are needed multiple times -> calculate them once
        double T2 = T*T;
        double T3 = T2 * T;

        //double s0, double v0, double a0, double sT, double vT, double aT
        std::vector<double> solution(6, 0);

        if(polynomialDegree_ == 3) {
            solution[0] = 0;    // not needed for degree == 3, just for easier implementation afterwards
            solution[1] = 0;    // idem
            solution[2] = (startState.vel + endState.vel)/T2 + 2*(startState.pos - endState.pos)/T3;
            solution[3] = (-2*startState.vel - endState.vel)/T + 3*(-startState.pos + endState.pos)/T2;
            solution[4] = startState.vel;
            solution[5] = startState.pos;
        }

        if(polynomialDegree_ == 5) {
            // T^4 and T^5 are needed multiple times -> calculate them once
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

    SamplingPoint ExuCobotCartesianVelocityController::evaluatePolynomial(std::vector<double> &coef, double t){

        // t^2, t^3, t^4 and t^5 are needed multiple times -> calculate them once
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        SamplingPoint state;

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
    void ExuCobotCartesianVelocityController::logEvaluatedTrajectory(){

        evaluatedTrajectoryFile_ << rosTimeString_ << "," << logTimeString_ << ",";

        if(!logYonly_) {
            evaluatedTrajectoryFile_ << currentState_.x.pos << ",";
            evaluatedTrajectoryFile_ << currentState_.x.vel << ",";
            evaluatedTrajectoryFile_ << currentState_.x.acc << ",";
            evaluatedTrajectoryFile_ << currentState_.x.jerk << ",";
        }

        evaluatedTrajectoryFile_ << currentState_.y.pos << ",";
        evaluatedTrajectoryFile_ << currentState_.y.vel << ",";
        evaluatedTrajectoryFile_ << currentState_.y.acc << ",";
        evaluatedTrajectoryFile_ << currentState_.y.jerk;

        if(!logYonly_) {
            evaluatedTrajectoryFile_ << ",";
            evaluatedTrajectoryFile_ << currentState_.z.pos << ",";
            evaluatedTrajectoryFile_ << currentState_.z.vel << ",";
            evaluatedTrajectoryFile_ << currentState_.z.acc << ",";
            evaluatedTrajectoryFile_ << currentState_.z.jerk;
        }

        evaluatedTrajectoryFile_ << std::endl;

        std::string msg = rosTimeString_ + "," + logTimeString_ + ","
                + std::to_string(currentState_.x.pos) + ","
                + std::to_string(currentState_.x.vel) + ","
                + std::to_string(currentState_.x.acc) + ","
                + std::to_string(currentState_.x.jerk) + ","
                + std::to_string(currentState_.y.pos) + ","
                + std::to_string(currentState_.y.vel) + ","
                + std::to_string(currentState_.y.acc) + ","
                + std::to_string(currentState_.y.jerk) + ","
                + std::to_string(currentState_.z.pos) + ","
                + std::to_string(currentState_.z.vel) + ","
                + std::to_string(currentState_.z.acc) + ","
                + std::to_string(currentState_.z.jerk);

        evalTrajLogger_->log(msg);
    }

    void ExuCobotCartesianVelocityController::logCurrentPosition(const std::array<double, 16> &current_robot_state, const std::array< double, 7 > &current_joint_positions) {
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

    void ExuCobotCartesianVelocityController::logTrajectoryCreation(const SamplingPoint3 &startState, const SamplingPoint3 &endState){

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

        trajectoryCreationFile_ << segmentDuration_.toSec();

        trajectoryCreationFile_ << std::endl;
    }

    void ExuCobotCartesianVelocityController::logCoefficients(){

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

    void ExuCobotCartesianVelocityController::publishState(ros::Time now, const SamplingPoint3 &state){
        util::posVelAccJerk3dStamped msg;
        // write struct into message
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
        pubCurrentState_.publish(msg);
    }

    void ExuCobotCartesianVelocityController::update(const ros::Time &time,
                                           const ros::Duration &period) {

        // update timing variables
        elapsedTime_ += period;
        segmentTime_ += period;

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
        evalTrajLogger_->log(msg);//, LogLevel::Debug, timeStr);

        lastRosTime = time;
        lastLogTime = logTime_;
        #endif

        static bool started = false;
        static int counter = 0;

        // get current pose of manipulator
        std::array<double, 16> current_robot_state= velocityCartesianHandle_->getRobotState().O_T_EE_d;

        // when starting the controller, wait until position buffer is filled with [minimalPositionBufferSize_] values
        if(!started){
            if(getPositionBufferReserve() >= minimalPositionBufferSize_){
                started = true;
                std::cerr << "Buffer partly filled with " << getPositionBufferReserve() << " entries. Starting-permission granted." << std::endl;
            }
        }

        if(started){

            // if segmentDuration_ has passed (=last segment is finished), calc new trajectory
            if(segmentTime_ >= segmentDuration_){
                #if ENABLE_LOGGING
                generalLogFile_ << "new Trajectory needed" << std::endl;
                #endif

                // new trajectory can only be generated if next command is already available
                if(getPositionBufferReserve() >= 1){

                    // if endTime of last segment != now, some time has passed since finishing of last trajectory and needs to be recovered
                    overdueTime_ = segmentTime_ - segmentDuration_;

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

                    generalLogFile_ << "updating trajectory after " << segmentTime_ << " s" << std::endl;
                    trajectoryCreationFile2_ << rosTimeString_ << "," << logTimeString_ << ",";
                    #endif
                    updateTrajectory();

                    // reset segmentTime_ as new one starts now
                    // if overdueTime_ != 0, less time is available to finish current (new) trajectory
                    segmentTime_ = overdueTime_;
                    #if ENABLE_LOGGING
                    generalLogFile_ << "setting segmentTime_ to " << segmentTime_ << std::endl;
                    #endif
                }
                else { // if there is no further entry in positionBuffer_, keep last velocity
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
                    currentState_.x = {current_robot_state[12], currentState_.x.vel, 0};
                    currentState_.y = {current_robot_state[13], currentState_.y.vel, 0};
                    currentState_.z = {current_robot_state[14], currentState_.z.vel, 0};
                }
            }

            // if within segment_duration, calc new state
            // can't be "else" to above statement, as it also has to be executed if trajectory has just been updated
            if(segmentTime_ < segmentDuration_) {
                // calculate new positions, velocities and accelerations
                #if ENABLE_LOGGING
                generalLogFile_ << "evaluating trajectory" << std::endl;
                #endif
                currentState_.x = evaluatePolynomial(coefs_[0], segmentTime_.toSec());
                currentState_.y = evaluatePolynomial(coefs_[1], segmentTime_.toSec());
                currentState_.z = evaluatePolynomial(coefs_[2], segmentTime_.toSec());

                #if ENABLE_LOGGING
                // get current pose
                std::array<double, 7> current_joint_positions= velocityCartesianHandle_->getRobotState().q;
                logEvaluatedTrajectory();
                logCurrentPosition(current_robot_state, current_joint_positions);
                #endif

                // extract new velocities
                double vx = currentState_.x.vel;
                double vy = currentState_.y.vel;
                double vz = currentState_.z.vel;

                // for now, any rotation is neglected
                double wx, wy, wz;
                wx = 0;
                wy = 0;
                wz = 0;

                // update command
                currentCommand_ = {vx, vy, vz, wx, wy, wz};

                if(exitIfTheoreticalValuesExceedLimits_) {

                    // check if v, a and j are within absolute limits

                    // acceleration
                    double ax = currentState_.x.acc;
                    double ay = currentState_.y.acc;
                    double az = currentState_.z.acc;

                    // jerk
                    double jx = currentState_.x.jerk;
                    double jy = currentState_.y.jerk;
                    double jz = currentState_.z.jerk;

                    double jAbs = sqrt(jx * jx + jy * jy + jz * jz);
                    double aAbs = sqrt(ax * ax + ay * ay + az * az);
                    double vAbs = sqrt(vx * vx + vy * vy + vz * vz);
                    bool quit = false;

                    // add small constant because of numerical inaccuracy
                    if (jAbs > maxJ_trans_ + 0.0001) {
                        #if ENABLE_LOGGING
                        generalLogFile_ << "ERROR: Jerk too high" << std::endl;
                        #endif
                        std::cerr << "ERROR: Jerk too high: " << jAbs << std::endl;
                        quit = true;
                    }

                    if (aAbs > maxA_trans_ + 0.0001) {
                        #if ENABLE_LOGGING
                        generalLogFile_ << "ERROR: Acceleration too high" << std::endl;
                        #endif
                        std::cerr << "ERROR: Acceleration too high: " << aAbs << std::endl;
                        quit = true;
                    }

                    if (vAbs > maxV_trans_ + 0.0001) {
                        #if ENABLE_LOGGING
                        generalLogFile_ << "ERROR: Velocity too high" << std::endl;
                        #endif
                        std::cerr << "ERROR: Velocity too high: " << vAbs << std::endl;
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
                        exit(-1);
                    }

                    currentCommand_ = {vx, vy, vz, wx, wy, wz};
                }
            }

            // check if new command is possible
            // max velocity change is max_acc * dt
            double max_dv = maxA_trans_ * 0.001;
            for(int i = 0; i < 3; ++i){ // check for all three axes

                // velocity change
                int dv = currentCommand_[i]-lastCommand_[i];

                if(abs(dv) >= max_dv){
                    // if desired velocity change is too high, cap it

                    #if ENABLE_LOGGING
                    std::cerr << "Last v: " << lastCommand_[i] << "\t next v: " << currentCommand_[i];
                    std::cerr << "\tdv: " << dv;
                    std::cerr << "\texceeds limit dv of " << max_dv;
                    #endif

                    // velocity increasing or decreasing
                    int sign = dv > 0 ? 1 : -1;     // dv can't be 0, as then above if-statement would not be fulfilled

                    // apply max velocity change but not more
                    currentCommand_[i] = lastCommand_[i] + max_dv * sign;

                    #if ENABLE_LOGGING
                    std::cerr << "\t-> next v: " << currentCommand_[i] << std::endl;
                    #endif
                }
            }

            #if ENABLE_LOGGING
            // pass velocity to robot control and log it
            commandLogFile_ << rosTimeString_ << "," << logTimeString_ << ", " << currentCommand_[0] << ", " << currentCommand_[1] << ", " << currentCommand_[2] << std::endl;
            #endif

            // check if one of commanded velocities is NaN. Can't reproduce error but once I got a "FrankaHW::controlCallback: Got NaN command!" fatal error
            for(int i = 0; i < 6; ++i){
                if(isnan(currentCommand_[i])){
                    #if ENABLE_LOGGING
                    generalLogFile_ << "ERROR: Command [" << i << "] is NaN" << std::endl;
                    generalLogFile_ << "Segment time: " << segmentTime_ << "\tSegment duration: " << segmentDuration_ << "\toverdue time: " << overdueTime_ << std::endl;
                    #endif
                    exit(-1);
                }
            }

            // pass command to robot
            velocityCartesianHandle_->setCommand(currentCommand_);

            // save given command for next iteration
            lastCommand_ = currentCommand_;
        }

        #if ENABLE_LOGGING
        double updateTime = (ros::Time::now() - time).toSec();
        double updateTimePrintThreshold = 1e-4;
        if(updateTime > updateTimePrintThreshold) { // update call on average takes 3e-5 - 5e-5 seconds. just print it when significantly above
            generalLogFile_ << "Update call took over " << updateTimePrintThreshold << " s ( " << updateTime << " s)" << std::endl;
        }
        #endif

        // publish current pos (not pose, even if command and variables are called like that)
        geometry_msgs::PoseStamped msgPose;
        msgPose.header.stamp = logTime_;
        msgPose.pose.position.x = current_robot_state[12];
        msgPose.pose.position.y = current_robot_state[13];
        msgPose.pose.position.z = current_robot_state[14];
        pubCurrentPose_.publish(msgPose);
    }

    void ExuCobotCartesianVelocityController::updateReferencePoseCallback(const util::segmentCommand &msg) {

        if(positionBufferWritingIndex_ == positionBufferReadingIndex_){
            std::cerr << "ERROR: Position buffer full" << std::endl;
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: Position buffer full" << std::endl;
            #endif
            exit(-1);
        }

        // store received command in struct. Jerk is not passed / is zero
        SamplingPoint3 state;
        state.x = {msg.x.pos, msg.x.vel, msg.x.acc, 0.0};
        state.y = {msg.y.pos, msg.y.vel, msg.y.acc, 0.0};
        state.z = {msg.z.pos, msg.z.vel, msg.z.acc, 0.0};

        // store command in fifo queue
        positionBuffer_[positionBufferWritingIndex_] = {state, msg.dt};

        if(msg.dt <= 0){
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: dt of new segment must be >0" << std::endl;
            #endif
            std::cerr << "ERROR: dt of new segment must be >0" << std::endl;
            exit(-1);
        }

        // increase writing index
        positionBufferWritingIndex_ = (positionBufferWritingIndex_ + 1) % positionBufferLength_;

        // for analytics
        currentReference__[0] = msg.x.pos;
        currentReference__[1] = msg.y.pos;
        currentReference__[2] = msg.z.pos;
        currentReference__[3] = msg.dt;

        #if ENABLE_LOGGING
        referenceLogFile_ << rosTimeString_ << "," << logTimeString_ << ",";
        referenceLogFile_ << msg.x << "," << msg.y << "," << msg.z << "," << msg.dt << std::endl;
        #endif
    }

    // function to calculate cartesian between two 3d points (!!! without range check !!!)
    double cartesianDistance(std::vector<double> s1, std::vector<double> s2){
        double dx = s2[0] - s1[0];
        double dy = s2[1] - s1[1];
        double dz = s2[2] - s1[2];
        return sqrt(dx*dx + dy*dy + dz*dz);
    }

    // function to round state to desired precision
    void roundState(SamplingPoint &state, int precision){
        double multiplier = pow(10, precision);

        state.pos = round(state.pos * multiplier)/multiplier;
        state.vel = round(state.vel * multiplier)/multiplier;
        state.acc = round(state.acc * multiplier)/multiplier;
    }

    // rounds 3d state to desired precision
    void roundState3(SamplingPoint3 &state, int precision){
        roundState(state.x, precision);
        roundState(state.y, precision);
        roundState(state.z, precision);
    }

    void ExuCobotCartesianVelocityController::updateTrajectory() {

        // at last iteration segment_time was at e.g. 0.009 (if segmentDuration_ is 0.01)
        // last calculation for previous segment has still to be done in order to let new segment start from correct position
        currentState_.x = evaluatePolynomial(coefs_[0], segmentDuration_.toSec());
        currentState_.y = evaluatePolynomial(coefs_[1], segmentDuration_.toSec());
        currentState_.z = evaluatePolynomial(coefs_[2], segmentDuration_.toSec());

        #if ENABLE_LOGGING
        //publishState(logTime_, currentState_);
        logEvaluatedTrajectory();
        generalLogFile_ << "evaluating trajectory" << std::endl;
        #endif

        // get current robot state
        std::array<double, 16> current_robot_state = velocityCartesianHandle_->getRobotState().O_T_EE_d;

        // if dift is allowed or it is the first time a trajectory is generated, currentState_ must be set
        if (allowDrift_ || (currentState_.x.pos == 0 && currentState_.y.pos == 0 && currentState_.z.pos == 0)) {
            // let trajectory start with calcualted velocity and acceleration, but with current (measured) position
            currentState_.x.pos = current_robot_state[12];
            currentState_.y.pos = current_robot_state[13];
            currentState_.z.pos = current_robot_state[14];
        }

        // startState equals current state, except .pos might be taken from robot end-effector position
        SamplingPoint3 startState = currentState_;

        double distance = cartesianDistance({currentState_.x.pos, currentState_.y.pos, currentState_.z.pos},
                                            {current_robot_state[12], current_robot_state[13], current_robot_state[14]});

        // distance must not be bigger than maximal distance which can be covered in one step (0.001s) with max velocity
        // note: this condition turned out to be necessary when the controller breaks. E.g. when libfranka stops because
        // of "libfranka: Move command aborted: motion aborted by reflex!", the robot would stop working, but this
        // controller would carry on and spam error messages in console. So with this condition this controller stops as
        // well.
        if (distance >= maxV_trans_ * 0.001) {
            std::cerr << "[" << rosTimeString_ << "] ERROR: Robot and Controller not in sync! Cartesian distance: " << distance << " m" << std::endl;
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: Robot and Controller not in sync! Cartesian distance: " << distance << " m" << std::endl;
            #endif
            exit(-1);
        }

        // index of next positions
        int i1 = (positionBufferReadingIndex_ + 1) % positionBufferLength_; // next position

        if(positionBuffer_[i1].dt <= 0){
            #if ENABLE_LOGGING
            generalLogFile_ << "ERROR: Desired segment-time must be >0" << std::endl;
            #else
            std::cout << "ERROR: Desired segment-time must be >0" << std::endl;
            #endif
            exit(-1);
        }

        segmentDuration_ = ros::Duration(positionBuffer_[i1].dt);

        #if ENABLE_LOGGING
        generalLogFile_ << "new segmentDuration_ is " << segmentDuration_.toSec() << " s" << std::endl;
        #endif

        // calculate trajectory endstate
        SamplingPoint3 endState;

        // get endState._.pos from position buffer
        for(int c = 0; c < 3; ++c){     // for coordinates c = x, y, z
            endState[c].pos = positionBuffer_[i1][c].pos;
            endState[c].vel = positionBuffer_[i1][c].vel;
            endState[c].acc = positionBuffer_[i1][c].acc;
        }

        #if ENABLE_LOGGING
        trajectoryCreationFile2_ << startState.y.pos << ",\t" << endState.y.pos << ",\t";   // pos
        trajectoryCreationFile2_ << startState.y.vel << ",\t" << endState.y.vel << ",\t";   // vel
        trajectoryCreationFile2_ << startState.y.acc << ",\t" << endState.y.acc << ",\t";   // acc
        trajectoryCreationFile2_ << (endState.y.pos - startState.y.pos) << ",\t";           // dp1
        trajectoryCreationFile2_ << (endState.y.vel - startState.y.vel) << ",\t";           // dv
        trajectoryCreationFile2_ << (endState.y.acc - startState.y.acc) << ",\t";           // da
        trajectoryCreationFile2_ << segmentDuration_.toSec() << std::endl;
        #endif

        // calculate polynom coefficients
        coefs_[0] = calcCoefs(startState.x, endState.x, segmentDuration_.toSec());
        coefs_[1] = calcCoefs(startState.y, endState.y, segmentDuration_.toSec());
        coefs_[2] = calcCoefs(startState.z, endState.z, segmentDuration_.toSec());

        #if ENABLE_LOGGING
        logTrajectoryCreation(startState, endState);
        logCoefficients();
        #endif

        // for next segment
        positionBufferReadingIndex_ = (positionBufferReadingIndex_ + 1) % positionBufferLength_;
    }

    void ExuCobotCartesianVelocityController::stopping(const ros::Time & /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.

        #if ENABLE_LOGGING
        generalLogFile_.close();
        referenceLogFile_.close();
        commandLogFile_.close();
        evaluatedTrajectoryFile_.close();
        currentPositionFile_.close();
        trajectoryCreationFile_.close();
        coefficientsFile_.close();
        trajectoryCreationFile2_.close();
        #endif
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ExuCobotCartesianVelocityController,
        controller_interface::ControllerBase
)
