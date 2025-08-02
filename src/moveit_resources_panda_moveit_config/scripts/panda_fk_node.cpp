#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using std::placeholders::_1;
using namespace std;
using namespace Eigen;

class PandaFKNode : public rclcpp::Node {
public:
    PandaFKNode() : rclcpp::Node("panda_fk_node", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)) {
        // Declare parameters for joint angles (degrees)
        this->declare_parameter<double>("fk_joint1", 30.0);
        this->declare_parameter<double>("fk_joint2", -45.0);
        this->declare_parameter<double>("fk_joint3", 20.0);
        this->declare_parameter<double>("fk_joint4", -60.0);
        this->declare_parameter<double>("fk_joint5", 45.0);
        this->declare_parameter<double>("fk_joint6", 10.0);
        this->declare_parameter<double>("fk_joint7", 0.0);
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "panda_arm_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&PandaFKNode::timer_callback, this));
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }

    Matrix4d dh(double a, double alpha, double d, double theta) {
        Matrix4d T;
        T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
             sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
             0,           sin(alpha),            cos(alpha),             d,
             0,           0,                     0,                      1;
        return T;
    }

    void timer_callback() {
        // Always get latest parameter values (support dynamic updates)
        std::vector<std::string> param_names = {
            "fk_joint1", "fk_joint2", "fk_joint3", "fk_joint4", "fk_joint5", "fk_joint6", "fk_joint7"
        };
        vector<double> joint_angles_deg;
        for (const auto& name : param_names) {
            joint_angles_deg.push_back(this->get_parameter(name).as_double());
        }
        vector<double> joint_angles;
        for (auto deg : joint_angles_deg)
            joint_angles.push_back(deg2rad(deg));

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
        Vector3d pos = T.block<3,1>(0,3);
        RCLCPP_INFO(this->get_logger(), "FK: End-effector Position: [%.3f, %.3f, %.3f]", pos(0), pos(1), pos(2));
        // Prepare and publish JointTrajectory message
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = joint_angles;
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);
        msg.points.push_back(point);
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Joint trajectory sent to panda_arm_controller/joint_trajectory.");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PandaFKNode>());
    rclcpp::shutdown();
    return 0;
}
