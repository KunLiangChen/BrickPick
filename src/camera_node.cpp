#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <robomaster_msgs/msg/h264_packet.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/**
 * @brief CameraNode 负责从 Robomaster 机器人获取并分发音视频数据。
 * 
 * 主要职责：
 * 1. 初始化与机器人的连接。
 * 2. 接收 H.264 编码的视频流。
 * 3. 解码视频帧并发布为 sensor_msgs/Image。
 * 4. 发布相机内参信息 (CameraInfo)。
 */
class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        this->declare_parameter("resolution", 360); // 360, 540, 720
        this->declare_parameter("protocol", "tcp"); // tcp or udp
        this->declare_parameter("enable_raw", true);
        this->declare_parameter("enable_h264", false);

        // 获取参数
        int resolution = this->get_parameter("resolution").as_int();
        std::string protocol = this->get_parameter("protocol").as_string();

        RCLCPP_INFO(this->get_logger(), "正在启动 Camera 节点, 分辨率: %dp",resolution);

        // 初始化发布者
        if (this->get_parameter("enable_raw").as_bool()) {
            image_pub_ = this->create_publisher<sensor_msgs::Image>("camera/image_color", 10);
            camera_info_pub_ = this->create_publisher<sensor_msgs::CameraInfo>("camera/camera_info", 10);
        }

        if (this->get_parameter("enable_h264").as_bool()) {
            h264_pub_ = this->create_publisher<robomaster_msgs::msg::H264Packet>("camera/image_h264", 10);
        }

        // 初始化定时器（模拟数据采集，实际应由 SDK 回调触发）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // 约 30 fps
            std::bind(&CameraNode::timer_callback, this)
        );

        // TODO: 在此处集成 Robomaster SDK 初始化逻辑
        // 1. 初始化机器人连接
        // 2. 开启视频流采集
        // 3. 注册解码回调
    }

private:
    void timer_callback() {
        // 这里是模拟的图像采集逻辑。
        // 在实际应用中，这里应该是从 SDK 的回调函数中接收解码后的 OpenCV Mat。
        
        if (image_pub_) {
            // 模拟一个空的图像帧
            cv::Mat frame = cv::Mat::zeros(360, 640, CV_8UC3);
            cv::putText(frame, "Robomaster Stream", cv::Point(50, 50), 
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

            // 将 OpenCV Mat 转换为 ROS 2 Image 消息
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            msg->header.stamp = this->now();
            msg->header.frame_id = "camera_optical_frame";

            image_pub_->publish(*msg);
            
            // 发布相机信息（示例）
            publish_camera_info(msg->header);
        }
    }

    void publish_camera_info(const std_msgs::msg::Header & header) {
        auto info = sensor_msgs::msg::CameraInfo();
        info.header = header;
        // 可以在此处填充具体的相机内参（从 YAML 文件加载）
        camera_info_pub_->publish(info);
    }

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::Publisher<robomaster_msgs::msg::H264Packet>::SharedPtr h264_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
