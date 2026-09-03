// ============================================================
// imu_node - IMU 惯性测量单元数据采集节点
//
// 职责：发布标准 IMU 话题 /imu/data（sensor_msgs/Imu）与
//       /imu/temperature，为 SLAM（Cartographer / VINS）提供姿态数据。
//
// 两种数据源（参数 imu_source）：
//   - "sim"    模拟模式：无需硬件，发布一帧静止 IMU 读数
//              （重力竖直向上 z=9.81），用于无 IMU 时联调链路。
//   - "serial" 串口模式：真 IMU（MPU6050/BNO085 等）驱动骨架，
//              TODO 待接入。
//
// 坐标系：frame_id 默认 imu_link，与 urdf/vi_car.urdf 一致。
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <chrono>
#include <memory>
#include <string>

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("imu_node")
    {
        // ---------- 参数（与 config/imu_params.yaml 对应） ----------
        this->declare_parameter<std::string>("imu_source", "sim");    // sim | serial
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<int>("publish_rate", 50);             // Hz
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string source = this->get_parameter("imu_source").as_string();
        if (source == "sim")
        {
            RCLCPP_INFO(this->get_logger(), "IMU 数据源：sim 模拟模式（无硬件）");
        }
        else if (source == "serial")
        {
            // TODO: 接入真实串口 IMU（MPU6050/BNO085 等）：
            //   1) 打开 serial_port / baud_rate 串口
            //   2) 按传感器协议解析角速度/加速度/姿态
            //   3) 在 timer_callback 里填真实数据
            RCLCPP_WARN(this->get_logger(),
                "串口 IMU 驱动尚未实现，当前只发布模拟数据。"
                "请实现 imu_node.cpp 中 serial 分支。");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "未知 imu_source: %s（应为 sim 或 serial）", source.c_str());
        }

        // ---------- 话题 ----------
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);

        // ---------- 定时发布 ----------
        int rate = this->get_parameter("publish_rate").as_int();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / rate),
            std::bind(&ImuNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "imu_node 初始化完成，发布 /imu/data");
    }

private:
    void timer_callback()
    {
        auto msg = std::make_unique<sensor_msgs::msg::Imu>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = this->get_parameter("frame_id").as_string();

        // 静止读数（模拟模式；串口模式接入后替换为真实数据）
        msg->orientation.w = 1.0;                 // 单位四元数，无旋转
        msg->angular_velocity.x = 0.0;
        msg->angular_velocity.y = 0.0;
        msg->angular_velocity.z = 0.0;
        msg->linear_acceleration.x = 0.0;
        msg->linear_acceleration.y = 0.0;
        msg->linear_acceleration.z = 9.81;        // 重力竖直向上
        // 协方差置零表示"未知"，避免下游被误导
        msg->orientation_covariance[0] = -1.0;

        imu_pub_->publish(std::move(msg));

        auto temp = std::make_unique<sensor_msgs::msg::Temperature>();
        temp->header.stamp = this->get_clock()->now();
        temp->header.frame_id = this->get_parameter("frame_id").as_string();
        temp->temperature = 25.0;
        temp_pub_->publish(std::move(temp));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
