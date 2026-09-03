// ============================================================
// lidar_node - 激光雷达数据采集节点
//
// 职责：发布标准 2D 激光话题 /scan（sensor_msgs/LaserScan），
//       供 Cartographer 等 SLAM 算法使用。
//
// 两种数据源（参数 lidar_source）：
//   - "sim"   模拟模式：无需硬件即可生成一帧静态房间扫描，
//             用于无雷达时联调 SLAM 数据链路。
//   - "serial" 串口模式：真雷达驱动的骨架，TODO 待接入
//             （RPLIDAR 等型号的协议解析留给后续实现）。
//
// 坐标系：frame_id 默认 laser_link，与 urdf/vi_car.urdf 一致。
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>

class LidarNode : public rclcpp::Node
{
public:
    LidarNode() : Node("lidar_node"), rng_(std::random_device{}())
    {
        // ---------- 参数（与 config/lidar_params.yaml 对应） ----------
        this->declare_parameter<std::string>("lidar_source", "sim");      // sim | serial
        this->declare_parameter<std::string>("frame_id", "laser_link");
        this->declare_parameter<int>("scan_fps", 10);                     // 发布频率
        this->declare_parameter<double>("range_min", 0.15);
        this->declare_parameter<double>("range_max", 12.0);

        // 串口模式参数（真雷达接入时使用）
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        std::string source = this->get_parameter("lidar_source").as_string();
        if (source == "sim")
        {
            RCLCPP_INFO(this->get_logger(), "雷达数据源：sim 模拟模式（无硬件）");
        }
        else if (source == "serial")
        {
            // TODO: 接入真实串口雷达（如 RPLIDAR A1）：
            //   1) 打开 serial_port / baud_rate 串口
            //   2) 按雷达协议发送测距请求、解析响应包
            //   3) 在 timer_callback 里填真实的 ranges
            RCLCPP_WARN(this->get_logger(),
                "串口雷达驱动尚未实现，当前只发布空扫描。"
                "请实现 lidar_node.cpp 中 serial 分支。");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "未知 lidar_source: %s（应为 sim 或 serial）", source.c_str());
        }

        // ---------- 话题 ----------
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        // ---------- 定时发布 ----------
        int fps = this->get_parameter("scan_fps").as_int();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / fps),
            std::bind(&LidarNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "lidar_node 初始化完成，发布 /scan");
    }

private:
    void timer_callback()
    {
        auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = this->get_parameter("frame_id").as_string();

        // 完整一圈扫描（-pi ~ pi）
        msg->angle_min = -M_PI;
        msg->angle_max = M_PI;
        msg->angle_increment = M_PI / 180.0;   // 1° 一步，共 360 点
        msg->time_increment = 0.0;
        msg->scan_time = 1.0 / this->get_parameter("scan_fps").as_int();
        msg->range_min = this->get_parameter("range_min").as_double();
        msg->range_max = this->get_parameter("range_max").as_double();

        std::string source = this->get_parameter("lidar_source").as_string();
        if (source == "sim")
        {
            msg->ranges = sim_scan();
        }
        else
        {
            // TODO: 从真实串口雷达读取数据填入 ranges
            msg->ranges.resize(360, msg->range_max);
        }

        scan_pub_->publish(std::move(msg));
    }

    // 模拟扫描：以机器人为中心、6m x 6m 的方形房间四壁的测量距离 + 噪声
    std::vector<float> sim_scan()
    {
        constexpr double kHalfWall = 3.0;   // 半墙长，房间 6m x 6m
        std::vector<float> ranges;
        ranges.reserve(360);
        std::normal_distribution<double> noise(0.0, 0.02);   // 2cm 高斯噪声

        for (int i = 0; i < 360; ++i)
        {
            double angle = -M_PI + i * (M_PI / 180.0);
            // 在 4 条边上求与射线 (cos, sin) 的交点距离，取最近者
            double dx = std::cos(angle);
            double dy = std::sin(angle);
            double d = std::numeric_limits<double>::infinity();
            if (dx > 1e-9) d = std::min(d, kHalfWall / dx);
            if (dx < -1e-9) d = std::min(d, -kHalfWall / dx);
            if (dy > 1e-9) d = std::min(d, kHalfWall / dy);
            if (dy < -1e-9) d = std::min(d, -kHalfWall / dy);
            double range = std::isinf(d) ? this->get_parameter("range_max").as_double()
                                         : d + noise(rng_);
            ranges.push_back(static_cast<float>(range));
        }
        return ranges;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    std::mt19937 rng_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}
