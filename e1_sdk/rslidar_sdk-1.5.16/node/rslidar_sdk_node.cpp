/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include "manager/node_manager.hpp"

#include <rs_driver/macro/version.hpp>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>


using namespace robosense::lidar;
// 使用原子变量标记是否请求停止
std::atomic<bool> g_shutdown_requested{false};

// 信号处理函数
static void sigHandler(int sig) {
  RS_MSG << "RoboSense-LiDAR-Driver is stopping....." << RS_REND;
  // 设置原子标志为true，表示请求关闭
  g_shutdown_requested.store(true, std::memory_order_relaxed);  // 原子操作，设置停止标志
}


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"



namespace robosense {
namespace lidar {
class Components_DriverNode : public rclcpp::Node {
public:
  Components_DriverNode(): Components_DriverNode(rclcpp::NodeOptions())
  {
    return;
  };

  Components_DriverNode(const rclcpp::NodeOptions & options)
    : Node("rslidar_sdk_components", options) {
      // 绑定 SIGINT (Ctrl+C) 信号，触发时调用信号处理函数
    signal(SIGINT, sigHandler);

    
     std::string config_path = std::string(PROJECT_PATH) + "/config/config.yaml";
    YAML::Node config;
    
    try {
      config = YAML::LoadFile(config_path);
    } catch (const YAML::Exception &e) { 
      RS_ERROR << "Error loading config file: " << e.what() << RS_REND;
      throw std::runtime_error("Config file loading failed.");
    }

    std::shared_ptr<NodeManager> demo_ptr = std::make_shared<NodeManager>();
    demo_ptr->init(config);
    demo_ptr->start();
    while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));  // 等待停止信号，避免阻塞
    }

    // 如果收到停止信号，进行清理操作
    demo_ptr->stop();  // 停止节点管理器
    RS_MSG << "Driver 停止完成。" << RS_REND;
  
  };

 virtual  ~Components_DriverNode(){};

};

}  // namespace lidar
}  // namespace robosense


RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::Components_DriverNode)
