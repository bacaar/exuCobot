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
        current_state_ = std::vector<std::vector<double>>(3, std::vector<double>(4, 0));    // for 3 dimensions a vector of size 4 (s, v, a, j)
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
        auto initial_pose = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

        // set next positions on current positions to stay here (NOT ZERO!!!)
        // TODO can I let this on zero because of my brilliant ring buffer logic?

        // set position entries of current_state_ vector to current positions
        current_state_[0][0] = initial_pose[12];
        current_state_[1][0] = initial_pose[13];
        current_state_[2][0] = initial_pose[14];

        segment_time_ = 0;
        lastSendingTime_ = ros::Time::now();

        elapsed_time_ = ros::Duration(0.0);

        stateFile_.open("/home/robocup/catkinAaron/src/exuCobot/state.log", std::ios::out);
        segmentFile_.open("/home/robocup/catkinAaron/src/exuCobot/segment.log", std::ios::out);

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

    std::vector<double> MyCartesianVelocityController::calcCoefs(double s0, double v0, double a0, double sT, double vT, double aT, double T){
        double T2 = T*T;
        double T3 = T2 * T;

        std::vector<double> solution(6, 0);

        if(polynomialDegree_ == 3) {
            solution[0] = 0;    // just for easier implementation afterwards
            solution[1] = 0;
            solution[2] = (v0 + vT)/T2 + 2*(s0 - sT)/T3;
            solution[3] = (-2*v0 - vT)/T + 3*(-s0 + sT)/T2;
            solution[4] = v0;
            solution[5] = s0;
            //[ds0/T**2 + dsT/T**2 + 2*s0/T**3 - 2*sT/T**3], [-2*ds0/T - dsT/T - 3*s0/T**2 + 3*sT/T**2], [ds0], [s0]]
        }

        if(polynomialDegree_ == 5) {
            double T4 = T3 * T;
            double T5 = T4 * T;

            solution[0] = -a0 / (2 * T3) + aT / (2 * T3) - 3 * v0 / T4 - 3 * vT / T4 - 6 * s0 / T5 + 6 * sT / T5;
            solution[1] = 3 * a0 / (2 * T2) - aT / T2 + 8 * v0 / T3 + 7 * vT / T3 + 15 * s0 / T4 - 15 * sT / T4;
            solution[2] = -3 * a0 / (2 * T) + aT / (2 * T) - 6 * v0 / T2 - 4 * vT / T2 - 10 * s0 / T3 + 10 * sT / T3;
            solution[3] = a0 / 2;
            solution[4] = v0;
            solution[5] = s0;
        }

        return solution;
    }

    std::vector<double> MyCartesianVelocityController::evaluatePolynom(std::vector<double> &coef, double t){

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;

        std::vector<double> stateVec(4, 0);

        stateVec[0] =    coef[0]*t5 +    coef[1]*t4 +   coef[2]*t3 +   coef[3]*t2 + coef[4]*t + coef[5];
        stateVec[1] =  5*coef[0]*t4 +  4*coef[1]*t3 + 3*coef[2]*t2 + 2*coef[3]*t  + coef[4];
        stateVec[2] = 20*coef[0]*t3 + 12*coef[1]*t2 + 6*coef[2]*t  + 2*coef[3];
        stateVec[3] = 60*coef[0]*t2 + 24*coef[1]*t  + 6*coef[2];

        return stateVec;

    }

    void MyCartesianVelocityController::logState(){
        auto time = ros::Time::now();
        stateFile_ << time.sec << ",";
        stateFile_ << time.nsec << ",";
        for(int coord = 0; coord < 3; ++coord)
            for(int derivative = 0; derivative < 4; ++derivative)
                stateFile_ << current_state_[coord][derivative] << ",";
        stateFile_ << std::endl;
    }

    void MyCartesianVelocityController::logSegment(){
        segmentFile_ << ros::Time::now().toSec();
        segmentFile_ << std::endl;
    }

    void publishState(const ros::Publisher &pub, const std::vector<std::vector<double>> &state){
        util::kinematicState3dStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.state.x.pos = state[0][0];
        msg.state.x.vel = state[0][1];
        msg.state.x.acc = state[0][2];
        msg.state.x.jerk = state[0][3];
        msg.state.y.pos = state[1][0];
        msg.state.y.vel = state[1][1];
        msg.state.y.acc = state[1][2];
        msg.state.y.jerk = state[1][3];
        msg.state.z.pos = state[2][0];
        msg.state.z.vel = state[2][1];
        msg.state.z.acc = state[2][2];
        msg.state.z.jerk = state[2][3];
        pub.publish(msg);
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
            if(getPositionBufferReserve() >= nominalPositionBufferSize_){
                started = true;
                std::cerr << "Buffer partly filled with " << getPositionBufferReserve() << " entries. Starting-permission granted.\n";
            }
        }

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

                    publishState(pub_current_state_, current_state_);
                    logState();

                    updateTrajectory();
                    logSegment();

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
                current_state_[0] = evaluatePolynom(coefs_[0], segment_time_);
                current_state_[1] = evaluatePolynom(coefs_[1], segment_time_);
                current_state_[2] = evaluatePolynom(coefs_[2], segment_time_);

                publishState(pub_current_state_, current_state_);
                logState();

                // when debugging
                if (max_segments != 0) {
                    //std::cerr << "[" << current_state_[1][0] << ", " << current_state_[1][1] << ", " << current_state_[1][2] << ", " << elapsed_time_.toSec() << "],\n";
                }

                // compose new velocity
                double vx = current_state_[0][1];
                double vy = current_state_[1][1];
                double vz = current_state_[2][1];

                double wx, wy, wz;
                wx = wy = wz = 0;

                // check jerk boundaries according to https://frankaemika.github.io/docs/control_parameters.html#limit-table
                {
                    double jx = current_state_[0][3];
                    double jy = current_state_[0][3];
                    double jz = current_state_[0][3];
                    double jAbs = sqrt(jx*jx + jy*jy + jz*jz);

                    const double max_j_trans = 6500.0; // m/s³

                    if (jAbs > max_j_trans) {
                        std::cerr << "ERROR: Jerk too high: " << jAbs << std::endl;
                        exit(-1);
                    }
                }

                // check acceleration boundaries
                {
                    double ax = current_state_[0][2];
                    double ay = current_state_[0][2];
                    double az = current_state_[0][2];
                    double aAbs = sqrt(ax*ax + ay*ay + az*az);

                    const double max_a_trans = 13.0; // m/s²

                    if (aAbs > max_a_trans) {
                        std::cerr << "ERROR: Acceleration too high: " << aAbs << std::endl;
                        exit(-1);
                    }
                }

                // check velocity boundaries
                {
                    double vAbs = sqrt(vx * vx + vy * vy + vz * vz);
                    const double max_v_trans = 1.7; // m/s
                    if (vAbs > max_v_trans) {
                        std::cerr << "ERROR: Velocity too high: " << vAbs << std::endl;
                        exit(-1);
                    }
                }

                // update command
                current_command_ = {vx, vy, vz, wx, wy, wz};
            }
            else{
                // do nothing, keep current_command_ the same as before
                // -> means that robot should keep current velocity
            }

            /*ros::Time now = ros::Time::now();
            double dif = (now - lastSendingTime_).toSec();
            if(dif > 0.0011){
                std::cerr << "dt: " << dif << std::endl;
            }*/

            // pass velocity to robot control
            velocity_cartesian_handle_->setCommand(current_command_);
            //lastSendingTime_ = now;
        }

        //rostopic pub -1 /franka_control/error_recovery/goal franka_msgs/ErrorRecoveryActionGoal "{}"
        //pub_error_recovery = node_handle.advertise<franka_msgs::ErrorRecoveryActionGoal>("{}", 20);

        // publish positions for python analytics
        geometry_msgs::PoseStamped msgPose;
        geometry_msgs::Vector3Stamped msgVec;

        msgPose.header.stamp = ros::Time::now();
        msgVec.header.stamp = msgPose.header.stamp;

        // current commanded velocity
        msgVec.vector.x = current_command_[0];
        msgVec.vector.y = current_command_[1];
        msgVec.vector.z = current_command_[2];
        pub_commanded_velocity_.publish(msgVec);

        // current position according to planned trajectory in cartesian space
        msgVec.vector.x = current_state_[0][0];
        msgVec.vector.y = current_state_[1][0];
        msgVec.vector.z = current_state_[2][0];
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
        msgnew.header.stamp = ros::Time::now();
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

        // get current pose
        std::array<double, 16> current_pose = velocity_cartesian_handle_->getRobotState().O_T_EE_d;

        // index of next positions
        int i1 = (position_buffer_index_reading_ + 1) % position_buffer_length_; // next position
        int i2 = (position_buffer_index_reading_ + 2) % position_buffer_length_; // second next position

        // calculate desired velocity for end of (first) segment as mean velocity of next two segments
        std::array<double, 3> next_velocity{};
        std::array<double, 3> next_acceleration{};

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

        next_acceleration[0] = (next_velocity[0] - current_state_[0][1])/segment_duration_;
        next_acceleration[1] = (next_velocity[1] - current_state_[1][1])/segment_duration_;
        next_acceleration[2] = (next_velocity[2] - current_state_[2][1])/segment_duration_;

        // calculate polynom coefficients
        // TODO: using current velocity and acceleration from current_state_ vector is not 100% correct, as they are the vel and acc from last step

        // std::cerr are for evaluating trajectory in pyhton, to control if calculation is correct
        if(testing_){
            coefs_[0] = calcCoefs(current_state_[0][0], current_state_[0][1], current_state_[0][2], position_buffer_[i1][0], next_velocity[0], next_acceleration[0], segment_duration_);
            coefs_[1] = calcCoefs(current_state_[1][0], current_state_[1][1], current_state_[1][2], position_buffer_[i1][1], next_velocity[1], next_acceleration[1], segment_duration_);
            coefs_[2] = calcCoefs(current_state_[2][0], current_state_[2][1], current_state_[2][2], position_buffer_[i1][2], next_velocity[2], next_acceleration[2], segment_duration_);
            //std::cerr << "[[" << current_state_[1][0];
        }
        else{
            coefs_[0] = calcCoefs(current_pose[12], current_state_[0][1], current_state_[0][2], position_buffer_[i1][0], next_velocity[0], next_acceleration[0], segment_duration_);
            coefs_[1] = calcCoefs(current_pose[13], current_state_[1][1], current_state_[1][2], position_buffer_[i1][1], next_velocity[1], next_acceleration[1], segment_duration_);
            coefs_[2] = calcCoefs(current_pose[14], current_state_[2][1], current_state_[2][2], position_buffer_[i1][2], next_velocity[2], next_acceleration[2], segment_duration_);
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

        stateFile_.close();
        segmentFile_.close();
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MyCartesianVelocityController,
        controller_interface::ControllerBase
)
