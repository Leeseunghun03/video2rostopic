#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>

class ImagePubNode : public rclcpp::Node
{
public:
    ImagePubNode() : Node("image_pub")
    {
        // Declare parameters
        this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
        // this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
        // this->declare_parameter<std::string>("camera_info_url", "");
        this->declare_parameter<std::string>("img_path", "");
        this->declare_parameter<std::string>("frame_id", "camera");
        this->declare_parameter<float>("pub_rate", 30.0f);
        this->declare_parameter<int>("start_sec", 0);
        this->declare_parameter<bool>("repeat", false);

        // Get parameters
        this->get_parameter("camera_topic", camera_topic_);
        // this->get_parameter("camera_info_topic", camera_info_topic_);
        // this->get_parameter("camera_info_url", camera_info_url_);
        this->get_parameter("img_path", img_path_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("pub_rate", pub_rate_);
        this->get_parameter("start_sec", start_sec_);
        this->get_parameter("repeat", repeat_);

        RCLCPP_INFO(this->get_logger(), "Camera Topic: %s", camera_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Image Path: %s", img_path_.c_str());

        // Camera Info Manager
        // camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, "camera", camera_info_url_);

        // Publishers
        img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_topic_, 10);
        // info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10);

        // Open video file
        video_capture_.open(img_path_);
        if (!video_capture_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", img_path_.c_str());
            throw std::runtime_error("Failed to open video file");
        }

        if (start_sec_ > 0)
        {
            video_capture_.set(cv::CAP_PROP_POS_MSEC, 1000.0 * start_sec_);
        }

        // Create timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / pub_rate_)),
            std::bind(&ImagePubNode::publish_frame, this));
    }

private:
    void publish_frame()
    {
        cv::Mat frame;
        if (!video_capture_.read(frame))
        {
            RCLCPP_WARN(this->get_logger(), "End of video reached.");
            if (repeat_)
            {
                video_capture_.open(img_path_);
                if (start_sec_ > 0)
                {
                    video_capture_.set(cv::CAP_PROP_POS_MSEC, 1000.0 * start_sec_);
                }
                return;
            }
            rclcpp::shutdown();
            return;
        }

        // Convert to RGB
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);

        // Convert to ROS Image message
        auto img_msg = sensor_msgs::msg::Image();
        img_msg.header.stamp = this->get_clock()->now();
        img_msg.header.frame_id = frame_id_;
        img_msg.height = frame.rows;
        img_msg.width = frame.cols;
        img_msg.encoding = "rgb8";
        img_msg.step = frame.cols * frame.channels();
        img_msg.data.assign(frame.data, frame.data + frame.total() * frame.elemSize());

        img_pub_->publish(img_msg);

        // // Publish camera info
        // if (camera_info_manager_->isCalibrated())
        // {
        //     auto info_msg = camera_info_manager_->getCameraInfo();
        //     info_msg.header = img_msg.header;
        //     info_pub_->publish(info_msg);
        // }
    }

    // Parameters
    std::string camera_topic_;
    // std::string camera_info_topic_;
    // std::string camera_info_url_;
    std::string img_path_;
    std::string frame_id_;
    float pub_rate_;
    int start_sec_;
    bool repeat_;

    // ROS 2 components
    // std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    // rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    cv::VideoCapture video_capture_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}