#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode() : Node("camera_node")
    {
        // Declare parameters with names matching camera_params.yaml
        this->declare_parameter<std::string>("camera_source", "usb");
        this->declare_parameter<int>("camera_id", 0);
        this->declare_parameter<int>("camera_width", 1920);
        this->declare_parameter<int>("camera_height", 1080);
        this->declare_parameter<int>("camera_fps", 30);
        this->declare_parameter<bool>("camera_hflip", false);
        this->declare_parameter<bool>("camera_vflip", false);
        this->declare_parameter<std::string>("camera_matrix_file", "calibration_data/camera_matrix.npy");
        this->declare_parameter<std::string>("dist_coeffs_file", "calibration_data/dist_coeffs.npy");

        // FPV specific parameters
        this->declare_parameter<std::string>("fpv_key", "gs.key");
        this->declare_parameter<int>("fpv_channel", 161);
        this->declare_parameter<int>("fpv_channel_width", 20);
        this->declare_parameter<std::string>("fpv_codec", "h265");
        this->declare_parameter<int>("fpv_width", 1920);
        this->declare_parameter<int>("fpv_height", 1080);
        this->declare_parameter<int>("fpv_fps", 60);

        // Get parameters
        std::string camera_source = this->get_parameter("camera_source").as_string();
        int camera_id = this->get_parameter("camera_id").as_int();
        int camera_width = this->get_parameter("camera_width").as_int();
        int camera_height = this->get_parameter("camera_height").as_int();
        int camera_fps = this->get_parameter("camera_fps").as_int();
        bool camera_hflip = this->get_parameter("camera_hflip").as_bool();
        bool camera_vflip = this->get_parameter("camera_vflip").as_bool();

        RCLCPP_INFO(this->get_logger(), "Camera source: %s", camera_source.c_str());

        // Initialize video capture
        if (camera_source == "usb")
        {
            RCLCPP_INFO(this->get_logger(), "Initializing USB camera with ID: %d", camera_id);
            cap_.open(camera_id);

            // Set camera properties
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);
            cap_.set(cv::CAP_PROP_FPS, camera_fps);
            cap_.set(cv::CAP_PROP_MODE, cv::CAP_MODE_VIDEO);

            // Set flip properties if supported
            cap_.set(cv::CAP_PROP_HFLIP, camera_hflip ? 1 : 0);
            cap_.set(cv::CAP_PROP_VFLIP, camera_vflip ? 1 : 0);
        }
        else if (camera_source == "fpv")
        {
            RCLCPP_INFO(this->get_logger(), "Initializing FPV camera on channel: %d",
                this->get_parameter("fpv_channel").as_int());

            // TODO: Initialize actual FPV receiver
            // For now, use dummy capture with FPV resolution
            int fpv_width = this->get_parameter("fpv_width").as_int();
            int fpv_height = this->get_parameter("fpv_height").as_int();
            int fpv_fps = this->get_parameter("fpv_fps").as_int();

            cap_.open(camera_id);
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, fpv_width);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, fpv_height);
            cap_.set(cv::CAP_PROP_FPS, fpv_fps);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown camera source: %s", camera_source.c_str());
            return;
        }

        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video capture device");
            return;
        }

        // Create publisher
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

        // Start timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / camera_fps),
            std::bind(&CameraNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Camera node initialized successfully");
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Failed to read frame from video capture");
            return;
        }

        // Flip frame if needed
        bool camera_hflip = this->get_parameter("camera_hflip").as_bool();
        bool camera_vflip = this->get_parameter("camera_vflip").as_bool();
        if (camera_hflip || camera_vflip)
        {
            cv::flip(frame, frame, (camera_hflip && camera_vflip) ? -1 : (camera_hflip ? 1 : 0));
        }

        // Convert to ROS message
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            this->get_clock()->now(), "bgr8", frame).toImageMsg();
        msg->header.frame_id = "camera_link";

        // Publish message
        image_pub_->publish(*msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}