#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

namespace tars_imu_tools {

class ImuTransform : public rclcpp::Node
{
public:
    ImuTransform(const rclcpp::NodeOptions& options) : Node("imu_transform", options)
    {
        using std::placeholders::_1;
        this->declare_parameter<std::string>("sub_topic", "/livox/imu");
        this->declare_parameter<std::string>("pub_topic", "/livox/imu_aft_tf");
        this->declare_parameter<double>("roll", 45.0);
        this->declare_parameter<double>("pitch", 0.0);
        this->declare_parameter<double>("yaw", 0.0);

        this->get_parameter<std::string>("sub_topic", sub_topic_);
        this->get_parameter<std::string>("pub_topic", pub_topic_);
        this->get_parameter<double>("roll", roll_);
        this->get_parameter<double>("pitch", pitch_);
        this->get_parameter<double>("yaw", yaw_);

        imu_sub_ = 
            this->create_subscription<sensor_msgs::msg::Imu>(
            sub_topic_, 10, 
            std::bind(&ImuTransform::imu_callback, this, _1));

        imu_pub_ = 
            this->create_publisher<sensor_msgs::msg::Imu>(pub_topic_, 10);

        init();
    }

private:
    void init() 
    {
        double cos_roll = cos(static_cast<double>(roll_ * M_PI / 180.0));
        double cos_pitch = cos(static_cast<double>(pitch_ * M_PI / 180.0));
        double cos_yaw = cos(static_cast<double>(yaw_ * M_PI / 180.0));
        double sin_roll = sin(static_cast<double>(roll_ * M_PI / 180.0));
        double sin_pitch = sin(static_cast<double>(pitch_ * M_PI / 180.0));
        double sin_yaw = sin(static_cast<double>(yaw_ * M_PI / 180.0));

        rotation[0][0] = cos_pitch * cos_yaw;
        rotation[0][1] = sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw;
        rotation[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

        rotation[1][0] = cos_pitch * sin_yaw;
        rotation[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
        rotation[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

        rotation[2][0] = -sin_pitch;
        rotation[2][1] = sin_roll * cos_pitch;
        rotation[2][2] = cos_roll * cos_pitch;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        sensor_msgs::msg::Imu tfed_imu = *imu_msg;

        tfed_imu.angular_velocity.x = imu_msg->angular_velocity.x * rotation[0][0] + 
                                      imu_msg->angular_velocity.y * rotation[0][1] +
                                      imu_msg->angular_velocity.z * rotation[0][2];
        
        tfed_imu.angular_velocity.y = imu_msg->angular_velocity.x * rotation[1][0] + 
                                      imu_msg->angular_velocity.y * rotation[1][1] +
                                      imu_msg->angular_velocity.z * rotation[1][2];
        
        tfed_imu.angular_velocity.z = imu_msg->angular_velocity.x * rotation[2][0] + 
                                      imu_msg->angular_velocity.y * rotation[2][1] +
                                      imu_msg->angular_velocity.z * rotation[2][2];

        tfed_imu.linear_acceleration.x = imu_msg->linear_acceleration.x * rotation[0][0] +
                                         imu_msg->linear_acceleration.y * rotation[0][1] +
                                         imu_msg->linear_acceleration.z * rotation[0][2];
        
        tfed_imu.linear_acceleration.y = imu_msg->linear_acceleration.x * rotation[1][0] +
                                         imu_msg->linear_acceleration.y * rotation[1][1] +
                                         imu_msg->linear_acceleration.z * rotation[1][2];

        tfed_imu.linear_acceleration.z = imu_msg->linear_acceleration.x * rotation[2][0] +
                                         imu_msg->linear_acceleration.y * rotation[2][1] +
                                         imu_msg->linear_acceleration.z * rotation[2][2];

        imu_pub_->publish(tfed_imu);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    double roll_;
    double pitch_;
    double yaw_;

    std::string sub_topic_;
    std::string pub_topic_;

    float rotation[3][3];

}; // class ImuTransform

}  // namespace tars_imu_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tars_imu_tools::ImuTransform)
