#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <sstream>
#include <iostream>
#include <chrono>
#include <ctime>
class ImageToVideo : public rclcpp::Node
{
public:
    ImageToVideo() : Node("image_to_video"), file_index_(0), frame_width_(0), frame_height_(0)
    {
        // Set up initial video file
        start_new_video_file();

        // Create a timer to switch files every minute
        timer_ = this->create_wall_timer(
            std::chrono::minutes(10),
            [this]() { start_new_video_file(); }
        );

        // Subscribe to image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_1/image_raw", 100000,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { this->image_callback(msg); }
        );
    }

private:
   void start_new_video_file()
{
    // 如果已有视频文件打开，则先关闭当前文件
    if (video_writer_.isOpened())
    {
        video_writer_.release(); // 关闭当前文件
        //RCLCPP_INFO(this->get_logger(), "已关闭当前视频文件");
    }

    // 生成新视频文件的文件名
  // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    
    // 转换为系统时间
    auto now_c = std::chrono::system_clock::to_time_t(now);
    
    // 将时间点转换为本地时间
    std::tm* now_tm = std::localtime(&now_c);
    
    // 输出时间
   // std::cout << "Current time: " << std::ctime(&now_c);
    int hour = now_tm->tm_hour;
    int minute = now_tm->tm_min;
    int day = now_tm->tm_mday;
    int month = now_tm->tm_mon + 1; 

    // std::cout << "Current day: " << day << std::endl;
    // std::cout << "Current month: " << month << std::endl;
    // std::cout << "Current hour: " << hour << std::endl;
    // std::cout << "Current minute: " << minute << std::endl;
    std::ostringstream str_time;
    std::ostringstream filename;
    str_time << month << "_" << day << "_" << hour << "_" << (minute/10)%10;
    //std::cout << "Filename: " << str_time.str() << std::endl;
    filename << "/home/nvidia/Videos/" << str_time.str() << ".avi"; // 使用 /tmp/ 进行测试

    // 如果尚未设置帧的宽度和高度，则不能打开视频文件
    if (frame_width_ == 0 || frame_height_ == 0)
    {
        //RCLCPP_WARN(this->get_logger(), "帧大小尚未设置，无法打开视频文件。");
        return;
    }

    cv::Size frame_size(frame_width_, frame_height_);
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // 使用 MJPG 编解码器
    double fps = 8.0; // 根据需要调整 FPS

    // 打开视频文件进行写入
    video_writer_.open(filename.str(), codec, fps, frame_size, true);

    // 检查视频文件是否成功打开
    if (!video_writer_.isOpened())
    {
       // RCLCPP_ERROR(this->get_logger(), "无法打开视频文件进行写入: %s", filename.str().c_str());
    }
    else
    {
       // RCLCPP_INFO(this->get_logger(), "视频文件成功打开: %s", filename.str().c_str());
    }
}

void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        // 将 ROS 图像消息转换为 OpenCV 图像
        cv::Mat cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (cv_image.empty())
        {
           // RCLCPP_WARN(this->get_logger(), "接收到空图像");
            return;
        }

        // 如果尚未设置帧的宽度和高度，则从图像中获取并初始化
        if (frame_width_ == 0 || frame_height_ == 0)
        {
            frame_width_ = cv_image.cols;
            frame_height_ = cv_image.rows;
            //RCLCPP_INFO(this->get_logger(), "图像大小设置为: %d x %d", frame_width_, frame_height_);
            start_new_video_file(); // 在设置帧大小后初始化视频写入器
        }

        // 确保视频写入器已打开
        if (!video_writer_.isOpened())
        {
           // RCLCPP_WARN(this->get_logger(), "视频写入器未打开。");
            return;
        }

        // 确保图像大小与视频写入器的帧大小匹配
        if (cv_image.cols != frame_width_ || cv_image.rows != frame_height_)
        {
           // RCLCPP_ERROR(this->get_logger(), "图像大小 (%d x %d) 与视频帧大小 (%d x %d) 不匹配。",
             //            cv_image.cols, cv_image.rows, frame_width_, frame_height_);
            return;
        }

        // 将图像写入视频文件
        video_writer_ << cv_image;
        //RCLCPP_INFO(this->get_logger(), "图像已写入: %d x %d", cv_image.cols, cv_image.rows);
    }
    catch (const cv_bridge::Exception &e)
    {
        //RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
    }
}


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    cv::VideoWriter video_writer_;
    rclcpp::TimerBase::SharedPtr timer_;
    int file_index_;
    int frame_width_;
    int frame_height_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageToVideo>());
    rclcpp::shutdown();
    return 0;
}
