#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>

using std::placeholders::_1;
using namespace std;
using namespace Eigen;

class PandaIKNode : public rclcpp::Node {
public:
    PandaIKNode() : Node("panda_ik_node") {
        // Declare parameters for target position and IK settings
        this->declare_parameter<double>("target_x", 0.5);
        this->declare_parameter<double>("target_y", 0.0);
        this->declare_parameter<double>("target_z", 0.4);
        this->declare_parameter<int>("max_iterations", 100);
        this->declare_parameter<double>("tolerance", 1e-3);
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "panda_arm_controller/joint_trajectory", 10);
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        target_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/target_position", qos_profile, std::bind(&PandaIKNode::target_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&PandaIKNode::timer_callback, this));
        // Initialize target position from parameters
        target_position_ = Vector3d(
            this->get_parameter("target_x").as_double(),
            this->get_parameter("target_y").as_double(),
            this->get_parameter("target_z").as_double()
        );
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        tolerance_ = this->get_parameter("tolerance").as_double();
        RCLCPP_INFO(this->get_logger(), "Panda IK Node initialized successfully!");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr target_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    Vector3d target_position_;
    bool new_target_received_ = false;
    int max_iterations_;
    double tolerance_;

    double deg2rad(double deg) { return deg * M_PI / 180.0; }
    double rad2deg(double rad) { return rad * 180.0 / M_PI; }

    // Dummy DH and FK for Panda (replace with real parameters for accuracy)
    Matrix4d dh(double a, double alpha, double d, double theta) {
        Matrix4d T;
        T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
             sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
             0,           sin(alpha),            cos(alpha),             d,
             0,           0,                     0,                      1;
        return T;
    }

    Matrix4d forward_kinematics(const vector<double>& joint_angles) {
        // Panda DH Parameters (approximate, for demonstration)
        vector<vector<double>> dh_params = {
            {0.0,  M_PI/2, 0.333, joint_angles[0]},
            {0.0, -M_PI/2, 0.0,   joint_angles[1]},
            {0.0,  M_PI/2, 0.316, joint_angles[2]},
            {0.0825, M_PI/2, 0.0, joint_angles[3]},
            {-0.0825, -M_PI/2, 0.384, joint_angles[4]},
            {0.0,  M_PI/2, 0.0, joint_angles[5]},
            {0.088, M_PI/2, 0.107, joint_angles[6]}
        };
        Matrix4d T = Matrix4d::Identity();
        for (auto &params : dh_params) {
            Matrix4d Ti = dh(params[0], params[1], params[2], params[3]);
            T = T * Ti;
        }
        return T;
    }

    MatrixXd compute_jacobian(const vector<double>& joint_angles) {
        MatrixXd J(3, 7); // 3D position jacobian for 7 joints
        double h = 1e-6;
        Vector3d pos_original = forward_kinematics(joint_angles).block<3,1>(0,3);
        for (int i = 0; i < 7; i++) {
            vector<double> perturbed_angles = joint_angles;
            perturbed_angles[i] += h;
            Vector3d pos_perturbed = forward_kinematics(perturbed_angles).block<3,1>(0,3);
            Vector3d derivative = (pos_perturbed - pos_original) / h;
            J.col(i) = derivative;
        }
        return J;
    }

    vector<double> solve_ik_newton_raphson(const Vector3d& target, const vector<double>& initial_guess,
                                          int max_iterations = 100, double tolerance = 1e-3) {
        vector<double> joint_angles = initial_guess;
        for (int iter = 0; iter < max_iterations; iter++) {
            Vector3d current_pos = forward_kinematics(joint_angles).block<3,1>(0,3);
            Vector3d error = target - current_pos;
            double error_norm = error.norm();
            if (error_norm < tolerance) {
                RCLCPP_INFO(this->get_logger(), "IK converged in %d iterations with error %.6f", iter, error_norm);
                return joint_angles;
            }
            MatrixXd J = compute_jacobian(joint_angles);
            JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV);
            MatrixXd J_pinv = svd.matrixV() * svd.singularValues().cwiseInverse().asDiagonal() * svd.matrixU().transpose();
            if (svd.singularValues().minCoeff() < 1e-6) {
                RCLCPP_WARN(this->get_logger(), "Near singular configuration detected at iteration %d", iter);
            }
            double damping = 0.01;
            MatrixXd J_damped = J.transpose() * (J * J.transpose() + damping * MatrixXd::Identity(3, 3)).inverse();
            VectorXd delta_q = J_damped * error;
            for (int i = 0; i < 7; i++) {
                joint_angles[i] += delta_q(i);
                joint_angles[i] = max(-2.7, min(2.7, joint_angles[i])); // Panda joint limits
            }
        }
        RCLCPP_WARN(this->get_logger(), "IK did not converge within %d iterations", max_iterations);
        return joint_angles;
    }

    void target_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        target_position_ = Vector3d(msg->x, msg->y, msg->z);
        new_target_received_ = true;
        RCLCPP_INFO(this->get_logger(), "✓ Target position received: [%.3f, %.3f, %.3f]", target_position_(0), target_position_(1), target_position_(2));
        solve_and_publish_ik();
    }
    void timer_callback() {
        // Dynamically update parameters
        target_position_ = Vector3d(
            this->get_parameter("target_x").as_double(),
            this->get_parameter("target_y").as_double(),
            this->get_parameter("target_z").as_double()
        );
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        tolerance_ = this->get_parameter("tolerance").as_double();
        if (!new_target_received_) {
            RCLCPP_INFO(this->get_logger(), "Using parameter target position: [%.3f, %.3f, %.3f]", target_position_(0), target_position_(1), target_position_(2));
            RCLCPP_INFO(this->get_logger(), "To send a new target: ros2 topic pub /target_position geometry_msgs/msg/Point \"{x: 0.5, y: 0.2, z: 0.4}\"");
        }
        solve_and_publish_ik();
        new_target_received_ = false;
    }
    void solve_and_publish_ik() {
        vector<double> initial_guess = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        vector<double> solution = solve_ik_newton_raphson(target_position_, initial_guess, max_iterations_, tolerance_);
        Vector3d achieved_pos = forward_kinematics(solution).block<3,1>(0,3);
        Vector3d final_error = target_position_ - achieved_pos;
        double final_error_norm = final_error.norm();
        RCLCPP_INFO(this->get_logger(), "Target: [%.3f, %.3f, %.3f]", target_position_(0), target_position_(1), target_position_(2));
        RCLCPP_INFO(this->get_logger(), "Achieved: [%.3f, %.3f, %.3f]", achieved_pos(0), achieved_pos(1), achieved_pos(2));
        RCLCPP_INFO(this->get_logger(), "Final error: %.6f", final_error_norm);
        RCLCPP_INFO(this->get_logger(), "Joint angles (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    rad2deg(solution[0]), rad2deg(solution[1]), rad2deg(solution[2]),
                    rad2deg(solution[3]), rad2deg(solution[4]), rad2deg(solution[5]), rad2deg(solution[6]));
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = solution;
        point.time_from_start = rclcpp::Duration::from_seconds(3.0);
        msg.points.push_back(point);
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "IK solution sent to panda_arm_controller/joint_trajectory.");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PandaIKNode>());
    rclcpp::shutdown();
    return 0;
}
