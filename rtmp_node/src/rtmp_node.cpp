#include "rtmp/rtmp_node.hpp"
#include <cv_bridge/cv_bridge.h>
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}
#include <chrono>
#include <ctime>
int64_t timestamp = 0;
ImageSubscriber::ImageSubscriber() : Node("image_subscriber") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera_0/image_raw", 1, std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

    avformat_network_init();

    const char *output_url = "rtmp://39.106.60.229/hls/LCFCHBHE1P1010046";
     //const char *output_url = "rtmp://192.168.10.106/live/image";
    avformat_alloc_output_context2(&fmt_ctx, nullptr, "flv", output_url);
    if (!fmt_ctx) {
        RCLCPP_ERROR(this->get_logger(), "Could not create output context");
        return;
    }

    AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        RCLCPP_ERROR(this->get_logger(), "Codec not found");
        return;
    }

    stream = avformat_new_stream(fmt_ctx, codec);
    if (!stream) {
        RCLCPP_ERROR(this->get_logger(), "Could not allocate stream");
        return;
    }

    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        RCLCPP_ERROR(this->get_logger(), "Could not allocate video codec context");
        return;
    }

    codec_ctx->codec_id = codec->id;
    codec_ctx->bit_rate = 400000;
    codec_ctx->width = 728;
    codec_ctx->height = 410;
    codec_ctx->time_base = {1, 10};
    codec_ctx->gop_size = 1;
    codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx->framerate = av_make_q(15, 1);

    if (fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not open codec");
        return;
    }

    if (avcodec_parameters_from_context(stream->codecpar, codec_ctx) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not copy codec context to stream parameters");
        return;
    }

    if (!(fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        if (avio_open(&fmt_ctx->pb, output_url, AVIO_FLAG_WRITE) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Could not open output URL '%s'", output_url);
            return;
        }
    }

    if (avformat_write_header(fmt_ctx, nullptr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error occurred when opening output URL");
        return;
    }
}

void ImageSubscriber::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    encode_and_stream(frame);
}

int64_t generate_timestamp_based_on_time() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

void ImageSubscriber::encode_and_stream(cv::Mat &frame) {
    AVFrame *av_frame = av_frame_alloc();
    av_frame->format = codec_ctx->pix_fmt;
    av_frame->width = codec_ctx->width;
    av_frame->height = codec_ctx->height;

    int ret = av_image_alloc(av_frame->data, av_frame->linesize, codec_ctx->width, codec_ctx->height, codec_ctx->pix_fmt, 32);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not allocate raw picture buffer");
        return;
    }

    struct SwsContext *sws_ctx = sws_getContext(
        frame.cols, frame.rows, AV_PIX_FMT_BGR24,
        codec_ctx->width, codec_ctx->height, codec_ctx->pix_fmt,
        SWS_BILINEAR, nullptr, nullptr, nullptr);
    
    uint8_t *in_data[1] = { frame.data };
    int in_linesize[1] = { static_cast<int>(frame.step) };

    sws_scale(sws_ctx, in_data, in_linesize, 0, frame.rows, av_frame->data, av_frame->linesize);

    //av_frame->pts = av_frame->best_effort_timestamp;
    //av_frame->pts =  generate_timestamp_based_on_time();
    av_frame->pts = timestamp++;
    AVPacket pkt = { 0 };
    av_init_packet(&pkt);
    pkt.data = nullptr; // packet data will be allocated by the encoder
    pkt.size = 0;
    ret = avcodec_send_frame(codec_ctx, av_frame);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error sending a frame for encoding");
        return;
    }

    ret = avcodec_receive_packet(codec_ctx, &pkt);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error during encoding");
        return;
    }
    //std::cout<<av_frame->pts<<std::endl;
    pkt.stream_index = stream->index;
    av_packet_rescale_ts(&pkt, codec_ctx->time_base, stream->time_base);

    ret = av_interleaved_write_frame(fmt_ctx, &pkt);
    if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error while writing packet");
        return;
    }

    av_packet_unref(&pkt);
    av_frame_free(&av_frame);
    sws_freeContext(sws_ctx);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
