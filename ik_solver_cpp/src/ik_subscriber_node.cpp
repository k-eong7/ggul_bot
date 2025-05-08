#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

class IKSubscriber
{
public:
    IKSubscriber(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        // 로봇 모델 로더 초기화 (URDF 로딩)
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");

        robot_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

        planning_group_ = "manipulator7";  
        joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);

        // 퍼블리셔 추가
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/target_pose", 10,
            std::bind(&IKSubscriber::poseCallback, this, std::placeholders::_1));

        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&IKSubscriber::jointStateCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(node_->get_logger(), "IK Subscriber initialized");
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joint_state_ = msg;
    }
    
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Received pose, calculating IK...");

        bool found_ik = robot_state_->setFromIK(joint_model_group_, *msg);

        if (found_ik)
        {
            std::vector<double> joint_values;
            robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
            RCLCPP_INFO(node_->get_logger(), "IK Solution:");

            // JointState 메시지 생성
            sensor_msgs::msg::JointState joint_state_msg;
            joint_state_msg.header.stamp = node_->get_clock()->now();

            // 조인트 이름을 0부터 5까지로 설정
            joint_state_msg.name = {
                "joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"
            };
            
            // IK로 계산된 joint_values를 설정
            joint_state_msg.position = joint_values;  // 계산된 IK 값

            // /joint_states 퍼블리시
            joint_state_pub_->publish(joint_state_msg);

            // IK 결과 출력
            for (size_t i = 0; i < joint_values.size(); ++i)
            {
                RCLCPP_INFO(node_->get_logger(), "  Joint %zu: %.3f", i, joint_values[i]);
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "No IK solution found");
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;  // 퍼블리셔 추가
    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::string planning_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ik_subscriber_node");
    IKSubscriber ik(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
