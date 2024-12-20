#ifndef RTMP_NODE_HPP
#define RTMP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
extern "C" {
#include <libavformat/avformat.h>
}

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber();
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    void encode_and_stream(cv::Mat &frame);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    AVFormatContext *fmt_ctx = nullptr;
    AVCodecContext *codec_ctx = nullptr;
    AVStream *stream = nullptr;
};

#endif // RTMP_NODE_HPP
