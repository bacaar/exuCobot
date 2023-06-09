// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <fstream>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

    class ExuCobotCartesianImpedanceController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;

        void starting(const ros::Time &) override;

        void stopping(const ros::Time &) override;

        void update(const ros::Time &, const ros::Duration &period) override;

    private:
        // Saturation
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(
                const Eigen::Matrix<double, 7, 1> &tau_d_calculated,
                const Eigen::Matrix<double, 7, 1> &tau_J_d);  // NOLINT (readability-identifier-naming)

        std::unique_ptr <franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr <franka_hw::FrankaModelHandle> model_handle_;
        std::vector <hardware_interface::JointHandle> joint_handles_;

        int translationalStiffness_ = 3600;
        int rotationalStiffness_ = 80;
        int translationalDamping_ = 80;
        int rotationalDamping_ = 10;

        double filter_params_{0.005};
        double nullspace_stiffness_{20.0};
        double nullspace_stiffness_reference_{20.0};
        const double delta_tau_max_{1.0};
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
        Eigen::Matrix<double, 6, 6> cartesian_stiffness_reference_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_;
        Eigen::Matrix<double, 6, 6> cartesian_damping_reference_;
        Eigen::Matrix<double, 7, 1> q_d_nullspace_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        std::mutex position_and_orientation_d_reference_mutex_;
        Eigen::Vector3d position_d_reference_;
        Eigen::Quaterniond orientation_d_reference_;

        // Dynamic reconfigure
        std::unique_ptr <dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
                dynamic_server_compliance_param_;
        ros::NodeHandle dynamic_reconfigure_compliance_param_node_;

        void complianceParamCallback(franka_example_controllers::compliance_paramConfig &config,
                                     uint32_t level);

        void updateReferencePoseCallback(const geometry_msgs::PoseStamped &msg);

        ros::Time logTime_;
        std::string logTimeString_;     // string with time starting at beginning of program
        std::string rosTimeString_;     // string with time starting at ros::Time(0)

        // log files
        std::ofstream referenceLogFile_;
        std::ofstream currentPositionFile_;

        // for measuring required time [us] of update-method
        int tarray_[10000];
        int tmin_;
        int tmax_;
        int tIndexMax_;
        int tIndex_;

        // for measuring required time [us] for receiving new pose via ros topic
        int rarray_[10000];
        int rmin_;
        int rmax_;
        int rIndexMax_;
        int rIndex_;

        ros::Publisher pub_current_pose_; // publisher for current pose
        ros::Publisher pub_current_error_; // publisher for current pose
        ros::Publisher pub_current_reference_; // publisher for current registered reference position

        // TODO: variable name management: there is no difference between desired and reference pose... different names make it difficult to read/understand

        ros::Subscriber sub_equilibrium_pose_; // Equilibrium pose subscriber
        ros::Subscriber sub_desired_pose_;   // Subscriber for new desired pose

    };

}  // namespace franka_example_controllers
