#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using std::placeholders::_1;
using namespace std;
using namespace Eigen;

class FKNode : public rclcpp::Node {
public:
    FKNode() : Node("fk_node") {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/panda_arm_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(2s, std::bind(&FKNode::timer_callback, this));
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
        // Define joint angles in degrees
        vector<double> joint_angles_deg = {0.0, 0.0, 0.0, 0.0, 0.0, 1.571, 0.785};
        vector<double> joint_angles;

        for (auto deg : joint_angles_deg)
            joint_angles.push_back(deg2rad(deg));

        // DH Parameters {a, alpha, d, theta}
        vector<vector<double>> dh_params = {
            {0.0,  M_PI/2, 0.3, joint_angles[0]},
            {0.4,  0.0,    0.0, joint_angles[1]},
            {0.3,  0.0,    0.0, joint_angles[2]},
            {0.0,  M_PI/2, 0.2, joint_angles[3]},
            {0.0,  0.0,    0.1, joint_angles[4]}
        };

        Matrix4d T = Matrix4d::Identity();
        for (auto &params : dh_params) {
            Matrix4d Ti = dh(params[0], params[1], params[2], params[3]);
            T = T * Ti;
        }

        Vector3d pos = T.block<3,1>(0,3);
        RCLCPP_INFO(this->get_logger(), "FK: End-effector Position: [%.3f, %.3f, %.3f]",
                    pos(0), pos(1), pos(2));

        // Prepare and publish JointTrajectory message
        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.joint_names = {'panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7'};

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = joint_angles;
        point.time_from_start = rclcpp::Duration::from_seconds(2.0);

        msg.points.push_back(point);
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Joint trajectory sent to controller.");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FKNode>());
    rclcpp::shutdown();
    return 0;
}
