// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// modified by Aaron Bacher

#include <franka_example_controllers/exucobot_cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <franka_example_controllers/pseudo_inversion.h>

#include <chrono>

namespace franka_example_controllers {

    bool ExuCobotCartesianImpedanceController::init(hardware_interface::RobotHW *robot_hw,
                                                   ros::NodeHandle &node_handle){
        std::vector<double> cartesian_stiffness_vector;
        std::vector<double> cartesian_damping_vector;

        // set callback method for updating reference pose
        sub_desired_pose_ = node_handle.subscribe("referencePose", 20,
                              &ExuCobotCartesianImpedanceController::updateReferencePoseCallback, this,
                              ros::TransportHints().reliable().tcpNoDelay());

        // create publisher for current pose
        pub_current_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("currentPose", 20);

        // create publisher for current pose
        pub_current_error_ = node_handle.advertise<geometry_msgs::PoseStamped>("currentError", 20);

        // create publisher for current reference pose
        pub_current_reference_ = node_handle.advertise<geometry_msgs::PoseStamped>("currentReference", 20);

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR_STREAM("ExuCobotCartesianImpedanceController: Could not read parameter arm_id");
            return false;
        }
        std::vector <std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
            ROS_ERROR(
                    "ExuCobotCartesianImpedanceController: Invalid or no joint_names parameters provided, "
                    "aborting controller init!");
            return false;
        }

        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM(
                    "ExuCobotCartesianImpedanceController: Error getting model interface from hardware");
            return false;
        }
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                    model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException &ex) {
            ROS_ERROR_STREAM(
                    "ExuCobotCartesianImpedanceController: Exception getting model handle from interface: "
                            << ex.what());
            return false;
        }

        auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM(
                    "ExuCobotCartesianImpedanceController: Error getting state interface from hardware");
            return false;
        }
        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                    state_interface->getHandle(arm_id + "_robot"));
        } catch (hardware_interface::HardwareInterfaceException &ex) {
            ROS_ERROR_STREAM(
                    "ExuCobotCartesianImpedanceController: Exception getting state handle from interface: "
                            << ex.what());
            return false;
        }

        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM(
                    "ExuCobotCartesianImpedanceController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException &ex) {
                ROS_ERROR_STREAM(
                        "ExuCobotCartesianImpedanceController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        dynamic_reconfigure_compliance_param_node_ =
                ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

        dynamic_server_compliance_param_ = std::make_unique <
                                           dynamic_reconfigure::Server <
                                           franka_example_controllers::compliance_paramConfig >> (

                                                   dynamic_reconfigure_compliance_param_node_);
        dynamic_server_compliance_param_->setCallback(
                boost::bind(&ExuCobotCartesianImpedanceController::complianceParamCallback, this, _1, _2));

        position_d_.setZero();
        orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        position_d_reference_.setZero();
        orientation_d_reference_.coeffs() << 0.0, 0.0, 0.0, 1.0;

        cartesian_stiffness_.setZero();
        cartesian_damping_.setZero();

        tmin_ = 1000;
        tmax_ = 0;
        tIndexMax_ = 10000;
        tIndex_ = 0;

        rmin_ = 1000;
        rmax_ = 0;
        rIndexMax_ = 10000;
        rIndex_ = 0;

        return true;
    }

    void ExuCobotCartesianImpedanceController::starting(const ros::Time & /*time*/) {
        // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
        // to initial configuration
        franka::RobotState initial_state = state_handle_->getRobotState();
        // get jacobian
        std::array<double, 42> jacobian_array =
                model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        // convert to eigen
        Eigen::Map <Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

        // set equilibrium point to current state
        position_d_ = initial_transform.translation();
        orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
        position_d_reference_ = initial_transform.translation();
        orientation_d_reference_ = Eigen::Quaterniond(initial_transform.linear());

        referenceLogFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/referenceIC.csv", std::ios::out);
        currentPositionFile_.open("/home/robocup/catkinAaron/src/exuCobot/log/currentPositionIC.csv", std::ios::out);

        if(!referenceLogFile_.is_open()) { std::cerr << "WARNING: Could not create/open reference log file!\n"; }
        else {
            referenceLogFile_ << "rt,t,px,py,pz,dt\n";
        }

        if(!currentPositionFile_.is_open()) { std::cerr << "WARNING: Could not create open current position log file!\n"; }
        else {
            currentPositionFile_ << "rt,t,px,py,pz,q0,q1,q2,q3,q4,q5,q6\n";
        }

        // set nullspace equilibrium configuration to initial q
        q_d_nullspace_ = q_initial;
    }

    // function to format ros::Time as s.ns
    std::string getRosTimeStringImpedanceController(ros::Time time){

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

    void ExuCobotCartesianImpedanceController::update(const ros::Time &time,
                                                const ros::Duration &period) {

        auto start = std::chrono::high_resolution_clock::now();

        logTime_ += period;
        logTimeString_ = getRosTimeStringImpedanceController(logTime_);
        rosTimeString_ = getRosTimeStringImpedanceController(time);

        // get state variables
        franka::RobotState robot_state = state_handle_->getRobotState();
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        std::array<double, 42> jacobian_array =
                model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

        //std::cout << "HEeheheheh MY mod \n";
        //std::cout << position_d_ << std::endl << "----------------------" << std::endl;

        // convert to Eigen
        Eigen::Map <Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
        Eigen::Map <Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        Eigen::Map <Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map <Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map <Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
                robot_state.tau_J_d.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

        currentPositionFile_ << rosTimeString_ << "," << logTimeString_ << ",";
        currentPositionFile_ << position[0] << ",";
        currentPositionFile_ << position[1] << ",";
        currentPositionFile_ << position[2] << ",";
        currentPositionFile_ << q[0] << ",";
        currentPositionFile_ << q[1] << ",";
        currentPositionFile_ << q[2] << ",";
        currentPositionFile_ << q[3] << ",";
        currentPositionFile_ << q[4] << ",";
        currentPositionFile_ << q[5] << ",";
        currentPositionFile_ << q[6];
        currentPositionFile_ << std::endl;

        // publish current cartesian position and orientation
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.pose.position.x = position[0];        // TODO maybe prettier with tf function?
        msg.pose.position.y = position[1];
        msg.pose.position.z = position[2];
        msg.pose.orientation.x = orientation.x();
        msg.pose.orientation.y = orientation.y();
        msg.pose.orientation.z = orientation.z();
        msg.pose.orientation.w = orientation.w();
        pub_current_pose_.publish(msg);

        /*
        // publish current reference pose
        msg.pose.position.x = position_d_reference_[0];
        msg.pose.position.y = position_d_reference_[1];
        msg.pose.position.z = position_d_reference_[2];
        msg.pose.orientation.x = orientation_d_reference_.x();
        msg.pose.orientation.y = orientation_d_reference_.y();
        msg.pose.orientation.z = orientation_d_reference_.z();
        msg.pose.orientation.w = orientation_d_reference_.w();
        pub_current_reference_.publish(msg);
         */

        // compute error to desired pose
        // position error
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d_;

        //std::cout << position << std::endl << "---------------------" << std::endl;

        // orientation error
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.linear() * error.tail(3);

        /*
        // publish current error of position and orientation
        msg.pose.position.x = error[0];        // TODO maybe prettier with tf function?
        msg.pose.position.y = error[1];
        msg.pose.position.z = error[2];
        msg.pose.orientation.x = error_quaternion.x();
        msg.pose.orientation.y = error_quaternion.y();
        msg.pose.orientation.z = error_quaternion.z();
        msg.pose.orientation.w = error_quaternion.w();
        pub_current_error_.publish(msg);
         */

        // compute control
        // allocate variables
        Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

        // pseudoinverse for nullspace handling
        // kinematic pseuoinverse
        Eigen::MatrixXd jacobian_transpose_pinv;
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

        // Cartesian PD control with damping ratio = 1
        tau_task << jacobian.transpose() *
                    (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
        // nullspace PD control with damping ratio = 1
        tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                          jacobian.transpose() * jacobian_transpose_pinv) *
                         (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                          (2.0 * sqrt(nullspace_stiffness_)) * dq);
        // Desired torque
        tau_d << tau_task + tau_nullspace + coriolis;
        // Saturate torque rate to avoid discontinuities
        tau_d << saturateTorqueRate(tau_d, tau_J_d);
        for (size_t i = 0; i < 7; ++i) {
            joint_handles_[i].setCommand(tau_d(i));
        }

        // update parameters changed online either through dynamic reconfigure or through the interactive
        // reference by filtering
        cartesian_stiffness_ =
                filter_params_ * cartesian_stiffness_reference_ + (1.0 - filter_params_) * cartesian_stiffness_;
        cartesian_damping_ =
                filter_params_ * cartesian_damping_reference_ + (1.0 - filter_params_) * cartesian_damping_;
        nullspace_stiffness_ =
                filter_params_ * nullspace_stiffness_reference_ + (1.0 - filter_params_) * nullspace_stiffness_;
        std::lock_guard <std::mutex> position_d_reference_mutex_lock(
                position_and_orientation_d_reference_mutex_);
        position_d_ = filter_params_ * position_d_reference_ + (1.0 - filter_params_) * position_d_;
        orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_reference_);

        if(false) { // toggle for analytics
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

            if (tIndex_ < tIndexMax_) {
                tarray_[tIndex_++] = (double) duration.count();
            } else if (tIndex_ == tIndexMax_) {

                // calculate mean value
                int sum = 0;
                for (int i = 0; i < tIndexMax_; ++i) {
                    int buf = tarray_[i];
                    sum += buf;
                    if (buf < tmin_) tmin_ = buf;
                    if (buf > tmax_) tmax_ = buf;
                }

                double mean = (double) sum / tIndexMax_;

                // calculate std deviation
                double stddev = 0;
                for (int i = 0; i < tIndexMax_; ++i) {
                    stddev += pow(tarray_[i] - mean, 2);
                }
                stddev /= (tIndexMax_ - 1);
                stddev = sqrt(stddev);

                std::cout << "UPDATE\t\tMin: " << tmin_ << "\tmean: " << mean << "\tMax: " << tmax_ << "\tStd Dev: "
                          << stddev << std::endl;

                tmin_ = 1000;
                tmax_ = 0;
                tIndex_ = 0;
            }
        }
    }

    void ExuCobotCartesianImpedanceController::updateReferencePoseCallback(const geometry_msgs::PoseStamped &msg)
    {

        //convert geometry_msgs/Vector3 to Eigen::Vector3d
        //tf::vectorMsgToEigen(msg, position_d_);   // TODO use this function?! (prettier)

        /*
        position_d_reference_(0) = msg.pose.position.x;
        position_d_reference_(1) = msg.pose.position.y;
        position_d_reference_(2) = msg.pose.position.z;

        orientation_d_.coeffs() << msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w;
         */

        if(false) { // toggle for analytics
            ros::Duration duration = ros::Time::now() - msg.header.stamp;
            int t = duration.nsec / 1000;

            if (rIndex_ < rIndexMax_) {
                rarray_[rIndex_++] = t;
            } else if (rIndex_ == rIndexMax_) {

                // calculate mean value
                int sum = 0;
                for (int i = 0; i < rIndexMax_; ++i) {
                    int buf = rarray_[i];
                    sum += buf;
                    if (buf < rmin_) rmin_ = buf;
                    if (buf > rmax_) rmax_ = buf;
                }

                double mean = (double) sum / rIndexMax_;

                // calculate std deviation
                double stddev = 0;
                for (int i = 0; i < rIndexMax_; ++i) {
                    stddev += pow(rarray_[i] - mean, 2);
                }
                stddev /= (rIndexMax_ - 1);
                stddev = sqrt(stddev);

                std::cout << "ROS\t\tMin: " << rmin_ << "\tmean: " << mean << "\tMax: " << rmax_ << "\tStd Dev: "
                          << stddev << std::endl;

                rmin_ = 1000;
                rmax_ = 0;
                rIndex_ = 0;
            }
        }

        std::lock_guard <std::mutex> position_d_reference_mutex_lock(
                position_and_orientation_d_reference_mutex_);
        position_d_reference_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
        Eigen::Quaterniond last_orientation_d_reference(orientation_d_reference_);
        orientation_d_reference_.coeffs() << msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w;
        if (last_orientation_d_reference.coeffs().dot(orientation_d_reference_.coeffs()) < 0.0) {
            orientation_d_reference_.coeffs() << -orientation_d_reference_.coeffs();
        }

        referenceLogFile_ << rosTimeString_ << "," << logTimeString_ << ",";
        referenceLogFile_ << msg.pose.position.x << "," << msg.pose.position.y << "," << msg.pose.position.z << ",0.01\n";
    }

    Eigen::Matrix<double, 7, 1> ExuCobotCartesianImpedanceController::saturateTorqueRate(
            const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
            const Eigen::Matrix<double, 7, 1> &tau_J_d) {  // NOLINT (readability-identifier-naming)
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] =
                    tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }
        return tau_d_saturated;
    }

    void ExuCobotCartesianImpedanceController::complianceParamCallback(
            franka_example_controllers::compliance_paramConfig &config,
            uint32_t /*level*/) {

        cartesian_stiffness_reference_.setIdentity();
        cartesian_stiffness_reference_.topLeftCorner(3, 3)
                << translationalStiffness_ * Eigen::Matrix3d::Identity();
        cartesian_stiffness_reference_.bottomRightCorner(3, 3)
                << rotationalStiffness_ * Eigen::Matrix3d::Identity();
        cartesian_damping_reference_.setIdentity();
        // Damping ratio = 1
        cartesian_damping_reference_.topLeftCorner(3, 3)
                << translationalDamping_ * Eigen::Matrix3d::Identity();
        cartesian_damping_reference_.bottomRightCorner(3, 3)
                << rotationalDamping_ * Eigen::Matrix3d::Identity();
        nullspace_stiffness_reference_ = config.nullspace_stiffness;

        std::cout << "\nStiffness modified!\n";
        std::cout << "Cartesian stiffness:\n" << cartesian_stiffness_reference_ << std::endl;
        std::cout << "Cartesian damping:\n" << cartesian_damping_reference_ << std::endl;
        std::cout << "Nullspace Stiffnes reference: " << nullspace_stiffness_reference_ << std::endl;

        std::cout << std::endl;
    }

    void ExuCobotCartesianImpedanceController::stopping(const ros::Time & /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.

        referenceLogFile_.close();
        currentPositionFile_.close();
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ExuCobotCartesianImpedanceController,
        controller_interface::ControllerBase
)
