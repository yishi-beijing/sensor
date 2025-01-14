#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "mycan_msgs/msg/hwcan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <nlohmann/json.hpp>
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include "tier4_planning_msgs/msg/route_state.hpp"
#include <boost/beast.hpp>
#include <thread>
#include <mutex>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>


using namespace std;
namespace beast = boost::beast;   // from <boost/beast.hpp>
namespace http = beast::http;     // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;       // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;  // from <boost/asio/ip/tcp.hpp>

std::mutex ws_mutex;

const double pi = 3.14159265358979324;
const double a = 6378245.0;
const double ee = 0.00669342162296594323;
double wgLat=41.768628;
double wgLon=123.435442;

static bool outOfChina(double lat, double lon)
{
    if (lon < 72.004 || lon > 137.8347)
        return true;
    if (lat < 0.8293 || lat > 55.8271)
        return true;
    return false;
}


static double transformLat(double x, double y)
{
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
    return ret;
}


static double transformLon(double x, double y)
{
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
    return ret;
}


static void gps_transform( double wgLat, double wgLon, double& mgLat, double& mgLon)
{
if (outOfChina(wgLat, wgLon))
        {
            mgLat = wgLat;
            mgLon = wgLon;
            return;
        }
        double dLat = transformLat(wgLon - 105.0, wgLat - 35.0);
        double dLon = transformLon(wgLon - 105.0, wgLat - 35.0);
        double radLat = wgLat / 180.0 * pi;
        double magic = sin(radLat);
        magic = 1 - ee * magic * magic;
        double sqrtMagic = sqrt(magic);
        dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
        dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
        mgLat = wgLat + dLat;
        mgLon = wgLon + dLon;
}

static void get_time(char* buffer )
{
            //获取时间
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", std::localtime(&now_c));
}
class WebSocketNode : public rclcpp::Node {
public:
    WebSocketNode()
        : Node("web_socket_node"),
          io_context_(),
          ws_(io_context_),
          work_guard_(net::make_work_guard(io_context_)),
          shutdown_requested_(false)
    {

       //    wss://www.ykcel.com/websocket/car/LCFCHBHE1P1010046  @吕东忱 

        std::string host,port;   
        this->declare_parameter("host", "39.106.60.229");
        this->get_parameter("host", host);
        this->declare_parameter("port", "8088");
        this->get_parameter("port", port);
        connect_to_server(host, port);


        read_thread_ = std::thread([this]() { read_loop(); });

        // // Use a separate thread to run the io_context
        io_context_thread_ = std::thread([this]() { io_context_.run(); });

        publisher_ = this->create_publisher<std_msgs::msg::String>("ws_out", 10);
        ws_publisher_ = this->create_publisher<std_msgs::msg::String>("ws_in", 10);
        hw_work_publisher_ = this->create_publisher<std_msgs::msg::Int32>("hw_work_state", 10);
        hw_mode_publisher_ = this->create_publisher<std_msgs::msg::Int32>("hw_mode_state", 10);
        planning_goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 10);
    
       
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "ws_in", 10, std::bind(&WebSocketNode::send_data, this, std::placeholders::_1));
        ws_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "ws_out", 10, std::bind(&WebSocketNode::read_data, this, std::placeholders::_1));
        hwcan_subscription_ = this->create_subscription<mycan_msgs::msg::Hwcan>(
        "hw_can_msgs", 10, std::bind(&WebSocketNode::ws_hw_msg_data, this, std::placeholders::_1));
        sub_gps_cmd = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/sensing/gnss/ublox/nav_sat_fix", 10, std::bind(&WebSocketNode::gps_callback, this, std::placeholders::_1));
        autoware_mode_cmd = this->create_subscription<tier4_planning_msgs::msg::RouteState>(
        "/planning/mission_planning/route_selector/main/state", 10, std::bind(&WebSocketNode::autoware_mode, this, std::placeholders::_1));
        websocket_init();
    }

    ~WebSocketNode() {
        shutdown_requested_ = true;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }

        // Gracefully close the WebSocket connection
        std::lock_guard<std::mutex> lock(ws_mutex_);
        beast::error_code ec;
        ws_.close(websocket::close_code::normal, ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Error closing WebSocket: %s", ec.message().c_str());
        }

        // Stop io_context and join its thread
        work_guard_.reset();
        io_context_.stop();
        if (io_context_thread_.joinable()) {
            io_context_thread_.join();
        }
    }

private:

    void autoware_mode(const tier4_planning_msgs::msg::RouteState::SharedPtr msg)
    {
        this->autoware_mode_msg=msg->state;
    }
    void ws_hw_msg_data(const mycan_msgs::msg::Hwcan::SharedPtr msg)
    {

         char msg_data[1024];
         int taskld=1;

         this->hw_msg=*msg;
         int driving_mode=this->hw_msg.can540.vehiclectrlmodefb;
         char time[48];
         double mgLat;
         double mgLon;
         int vehicleStatus=1;
         get_time(time);
         gps_transform(wgLat,wgLon,mgLat,mgLon);
         if (driving_mode==0)
         {
            driving_mode=2;
         }else if(driving_mode==2)  
         {
            driving_mode=1;
         }else{
            driving_mode=3;
         }
         //小程序测试
         driving_mode=1;
         
         int expectedRemainingDrivingTime=0;

        snprintf(msg_data, sizeof(msg_data),
                "{ \"type\": \"report\", \"data\" :{ \"taskld\":%d,\"battery\": %d, \"carNo\": \"%s\", \"cupulaWorkStatus\": %d, \"drivingMode\": %d, "
                "\"expectedRemainingDrivingTime\": %d, \"garbageBinOverflowStatus\": %d, "
                "\"latitude\": %lf, \"location\": \"%s\", \"longitude\": %lf, \"params\": \"%s\", "
                "\"remainingWaterPercentage\": %d, \"remark\": \"%s\", \"reportingTime\": \"%s\", "
                "\"speed\": \"%lf\", \"sweepWorkStatus\": %d, \"vehicleStatus\": %d, \"vin\": \"%s\", "
                "\"wateringWorkStatus\": %d } }",
                taskld,this->hw_msg.can535.chassispowersocfb, 
                car_no.c_str(),
                this->hw_msg.can540.workenablefb, 
                driving_mode,
                expectedRemainingDrivingTime, 
                this->hw_msg.can540.dustbinoverflowstatusfb,
                mgLat, 
                location.c_str(), 
                mgLon,
                params.c_str(), 
                this->hw_msg.can548.waterlevelratio,
                remark.c_str(),
                time, 
                (double)this->hw_msg.can530.chassisspeedfb/100*3.6, 
                 this->hw_msg.can540.workenablefb,
                vehicleStatus, 
                Vin.c_str(),
                this->hw_msg.can540.sweepwatersprayfb);

        //printf("%s\n", msg_data);
        auto report = std_msgs::msg::String();
        report.data = msg_data;  // 将C字符串赋值给消息的data成员
        ws_publisher_->publish(report);


        std::string warnName;
        if (this->hw_msg.can543.faultdiagnosis!=0)
        {
           warnName="底盘故障";
        }else if(this->hw_msg.can543.bty_systemfault!=0)
        {
           warnName="电池故障";
        }else if(this->hw_msg.can543.mcu_systemfault!=0)
        {
           warnName="驱动故障";
        }else{
            return;
        }

        snprintf(msg_data, sizeof(msg_data),
                "{ \"type\": \"reportWarn\", \"data\" :{ \"reportTime\":\"%s\",\"warnName\": \"%s\" } }",
                time,
                warnName.c_str());

        //printf("%s\n", msg_data);
        auto reportWarn = std_msgs::msg::String();
        reportWarn.data = msg_data;  // 将C字符串赋值给消息的data成员
        ws_publisher_->publish(reportWarn);


    }

   void read_data(const std_msgs::msg::String::SharedPtr msg) 
   {
    std::string json_data = msg->data;
    auto json = nlohmann::json::parse(json_data);
    std::string type_value = json["type"];
    long int serialNumber = json["serialNumber"];
    char msg_data[1024];
    int code=0;
    std::string message=" ok";
    // 确保 node 被正确声明和初始化
    //std::cout<<"sub : type :"<<type_value.c_str()<<std::endl;

    if (type_value=="actionAutoReq")
    {
         auto node = rclcpp::Node::make_shared("service_client_actionAutoReq_node"); 
        int data_value = json["data"];
        std_msgs::msg::Int32 work_data;
       // std::cout<<"data_value :"<<data_value<<std::endl;
        if (data_value==1)
        {
            work_data.data=data_value;
            hw_work_publisher_->publish(work_data);
           // sleep(3);
            auto client = node->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("api/operation_mode/change_to_autonomous");

    
            if (!client->service_is_ready()) {
                message= "Service is not available. Skipping call.";
                code=0;
            }else{

                auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
               
                auto future_result = client->async_send_request(request);

                if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS) {
                    //RCLCPP_INFO(node->get_logger(), "Service call succeeded.");
                    message= "Service call succeeded.";
                    code=1;
                } else {
                    //RCLCPP_ERROR(node->get_logger(), "Service call failed.");
                     message= "Service call failed.";
                    code=0;
                }
            }
       
        }else if(data_value==2)
        {     

            work_data.data=data_value;
            hw_work_publisher_->publish(work_data);
            auto client = node->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("api/operation_mode/change_to_stop");

            if (!client->service_is_ready()) {
                message= "Service is not available. Skipping call.";
                code=0;
            }else{

                auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
               
                auto future_result = client->async_send_request(request);

                if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS) {
                    //RCLCPP_INFO(node->get_logger(), "Service call succeeded.");
                    message= "Service call succeeded.";
                    code=1;
                } else {
                    //RCLCPP_ERROR(node->get_logger(), "Service call failed.");
                    message= "Service call failed.";
                    code=0;
                }
            }   

        } 

         snprintf(msg_data, sizeof(msg_data),
                "{ \"type\": \"actionAutoResp\", \"data\" :{ \"serialNumber\":\"%ld\",\"code\": \"%d\",\"message\": \"%s\"} }",
                serialNumber,
                code,
                message.c_str());
          //  printf("%s\n", msg_data);
              auto reportWarn = std_msgs::msg::String();
              reportWarn.data = msg_data;  // 将C字符串赋值给消息的data成员
              ws_publisher_->publish(reportWarn);
       
    }else if (type_value=="drivingModeReq")
    {
    

        std::thread taskReq_thread(std::bind(&WebSocketNode::drivingModeReq_func, this,json));
        taskReq_thread.detach();
       

    }else if (type_value=="taskReq")
    {

        std::thread taskReq_thread(std::bind(&WebSocketNode::taskReq_func, this, json));
        taskReq_thread.detach();
        
    }else if (type_value=="offlineVideoReq")
    {


        char msg_data[1024];
        char buffer[256];
        long int serialNumber = json["serialNumber"];
        std::string startTime = json["data"]["startTime"];
        std::string endTime = json["data"]["endTime"];

        
        int code = 1;
        std::string message = "ok";
        std::string rtmp_server_ = "rtmp://39.106.60.229/hls/LCFCHBHE1P1010046";
        std::string video_path = "/home/nvidia/Videos/9_1_13_3.avi";
        sprintf(buffer, "ffmpeg -re -i  %s -c:v libx264 -c:a aac -f flv  %s_%ld > /home/nvidia/1.log 2>&1", video_path.c_str(),rtmp_server_.c_str(),serialNumber);
        std::string ffmpeg_command = buffer;

        snprintf(msg_data, sizeof(msg_data),
                "{ \"type\": \"offlineVideoResp\", \"data\" :{ \"serialNumber\":\"%ld\",\"code\": \"%d\",\"message\": \"%s\",\"key\": \"%s_%ld\"} }",
        serialNumber,
        code,
        message.c_str(),
        Vin.c_str(),
        serialNumber);
       // printf("pub :%s\n", msg_data);
        auto offlineVideoResp = std_msgs::msg::String();
        offlineVideoResp.data = msg_data;  // 将C字符串赋值给消息的data成员
        
       
        std::thread offlineVideoReq_thread(std::bind(&WebSocketNode::offlineVideoReq_func, this, ffmpeg_command));
        offlineVideoReq_thread.detach();
        sleep(8);
        ws_publisher_->publish(offlineVideoResp);
      
    }else if(type_value=="taskStopReq"){
         char msg_data[1024];
         long int serialNumber = json["serialNumber"];
         int code=1;
        std::string message=" ok";
         snprintf(msg_data, sizeof(msg_data),
                "{ \"type\": \"taskStopResp\", \"data\" :{ \"serialNumber\":\"%ld\",\"code\": \"%d\",\"message\": \"%s\"} }",
        serialNumber,
        code,
        message.c_str());
       // printf("pub :%s\n", msg_data);
        auto offlineVideoResp = std_msgs::msg::String();
        offlineVideoResp.data = msg_data;  // 将C字符串赋值给消息的data成员

    }

     // std::cout<<"over"<<std::endl;
    
    }

    void drivingModeReq_func(nlohmann::json & json)
    {
        std::string type_value = json["type"];
        long int serialNumber = json["serialNumber"];
        char msg_data[1024];
        int code=0;
        std::string message=" ok";
        int data_value = json["data"];
        std_msgs::msg::Int32 mode_data;
     
        if (data_value==1)
        {
            mode_data.data=data_value;
            hw_mode_publisher_->publish(mode_data);
        }else if(data_value==2)
        {
            mode_data.data=data_value;
            hw_mode_publisher_->publish(mode_data);
        }
      
        sleep(3);
        int mode=this->hw_msg.can540.vehiclectrlmodefb;

        //std:cout<<mode<<endl;
         if (data_value==1&&mode==2)
        {
            code=1;
          
        }else if(data_value==2&&mode==0)
        {
            code=1;
           
        }else{
         message= "Switching driving mode failed";
        }
        snprintf(msg_data, sizeof(msg_data),
                "{ \"type\": \"drivingModeResp\", \"data\" :{ \"serialNumber\":\"%ld\",\"code\": \"%d\",\"message\": \"%s\"} }",
        serialNumber,
        code,
        message.c_str());
        auto reportWarn = std_msgs::msg::String();
        reportWarn.data = msg_data;  // 将C字符串赋值给消息的data成员
       //  printf("pub :%s\n", msg_data);
        ws_publisher_->publish(reportWarn);
    }

    void offlineVideoReq_func(std::string & ffmpeg_command)
    {
        std::cout<<ffmpeg_command<<std::endl;
        int result = std::system(ffmpeg_command.c_str());
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute FFmpeg command");
        }
       // std::cout<<"over"<<std::endl;
    }
   

 void taskReq_func(nlohmann::json & json) 
   {
        //std::string route = json["data"]["route"];
        //std::string returnPoint = json["data"]["returnPoint"];
        //std::cout<<"route  "<<route<<std::endl;
        //std::cout<<"returnPoint  "<<returnPoint<<std::endl;
        char msg_data[1024];
        int code=0;
        long int serialNumber = json["serialNumber"];
        int taskId = json["data"]["taskId"];
        std::string route = json["data"]["route"];
        geometry_msgs::msg::PoseStamped pose;
        char time[48];
        std::string message=" ok";
        std_msgs::msg::Int32 work_data;


        
     //   work_data.data=1;
    //    hw_work_publisher_->publish(work_data);   
        pose.header.frame_id="map";
        pose.pose.position.x=poses_map[route].position.x;
        pose.pose.position.y=poses_map[route].position.y;
        pose.pose.position.z=poses_map[route].position.z;
        pose.pose.orientation.x=poses_map[route].orientation.x;
        pose.pose.orientation.y=poses_map[route].orientation.y;
        pose.pose.orientation.z=poses_map[route].orientation.z;
        pose.pose.orientation.w=poses_map[route].orientation.w;
        planning_goal_publisher_->publish(pose);

        sleep(3);


        auto node = rclcpp::Node::make_shared("service_client_taskReq_node");  // 确保 node 被正确声明和初始化
        auto client = node->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("api/operation_mode/change_to_autonomous");
    
            if (!client->service_is_ready()) {
                 return;
            }

            auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
               
            auto future_result = client->async_send_request(request);

            if (rclcpp::spin_until_future_complete(node, future_result) == rclcpp::FutureReturnCode::SUCCESS) {
                    message="Service call succeeded.";
                    code=1;
                 
            } else {
                    message="Service call failed.";
            
            }


        get_time(time);
        snprintf(msg_data, sizeof(msg_data),
                "{ \"type\": \"taskStartResp\", \"data\" :{ \"serialNumber\":\"%ld\",\"taskId\": \"%d\",\"startTime\": \"%s\",\"code\": \"%d\",\"message\": \"%s\"} }",
        serialNumber,
        taskId,
        time,
        code,
        message.c_str());
        //std::cout<<"msg_data : "<<msg_data<<std::endl;
    //    printf("pub :%s\n", msg_data);
        auto taskStartResp = std_msgs::msg::String();
        taskStartResp.data = msg_data;  // 将C字符串赋值给消息的data成员
        ws_publisher_->publish(taskStartResp);   
        sleep(1);
       // 任務結束
        get_time(time);
        snprintf(msg_data, sizeof(msg_data),
                "{ \"type\": \"taskEndResp\", \"data\" :{ \"serialNumber\":\"%ld\",\"taskId\": \"%d\",\"endTime\": \"%s\",\"code\": \"%d\",\"message\": \"%s\",\"area\": \"1221.22\",\"powerConsumption\": \"1223.22\"} }",
        serialNumber,
        taskId,
        time,
        code,
        message.c_str());
       // std::cout<<"msg_data : "<<msg_data<<std::endl;
        auto taskEndResp = std_msgs::msg::String();
        taskEndResp.data = msg_data;  // 将C字符串赋值给消息的data成员
        ws_publisher_->publish(taskEndResp);
 
   }


    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
    sensor_msgs::msg::NavSatFix gps=*msg;
    wgLat=gps.latitude;
    wgLon=gps.longitude;
    // std::cout<<wgLat<<std::endl;
    // std::cout<<wgLon<<std::endl;
    }

private:
    void connect_to_server(const std::string& host, const std::string& port) 
    {


       // std::string host = "example.com";  // 替换为你选择的 WebSocket 服务器地址
       // std::string port = "80";           // 或者 "443" 如果你使用的是 WebSocket over HTTPS
      
        tcp::resolver resolver(io_context_);
        auto const results = resolver.resolve(host.c_str(), port.c_str());
        net::connect(ws_.next_layer(), results.begin(), results.end());
        std::cout<<"host  :  "<<host.c_str() <<"  port  ： "<<port.c_str()<<std::endl;
        try {

            //    wss://www.ykcel.com/websocket/car/LCFCHBHE1P1010046
            ws_.handshake(host.c_str(), "/websocket/car/LCFCHBHE1P1010046");
        } catch (const boost::system::system_error& e) {
            std::cerr << "Error during handshake: " << e.what() << std::endl;
            std::cerr << "Error Code: " << e.code() << std::endl;
        }

    }

    void read_loop() {
        rclcpp::WallRate loop_rate(5);
        //std::cout<<"read_loop  :  "<<std::endl;
        while (rclcpp::ok()) {
            beast::flat_buffer buffer;
            try {
                {  
                    ws_.read(buffer);
                }
                std_msgs::msg::String msg;
                msg.data = beast::buffers_to_string(buffer.data());
                 //std::cout<<"buffer  :  "<<msg.data.c_str()<<std::endl;
                publisher_->publish(msg);
                std::cout<<"read_loop  :  "<<msg.data.c_str()<<std::endl;
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Read error: %s", e.what());
                //std::cout<<"error   e"<<e.what()<<std::endl;
            }
            //std::cout<<"sleep  :  "<<std::endl;
           loop_rate.sleep();

        }
    }

    void send_data(const std_msgs::msg::String::SharedPtr msg) {
       
        std::lock_guard<std::mutex> lock(ws_mutex);
        ws_.async_write(net::buffer(msg->data),
            [this](beast::error_code ec, std::size_t /*bytes_transferred*/) {
                if (ec) {
                    RCLCPP_ERROR(this->get_logger(), "Send error: %s", ec.message().c_str());
                }
            });
         std::cout<<"send_data  : "<<msg->data<<std::endl;
    }
    
    void websocket_init()
    {

        char msg_data[1024]; 
        char temp_data[256];
        std::string yaml_file_path;
        this->declare_parameter("yaml_config", "default_value.yaml");
        this->get_parameter("yaml_config", yaml_file_path);
        std::cout<<yaml_file_path<<std::endl;
         YAML::Node config = YAML::LoadFile(yaml_file_path);
        int count = config["poses"].size();  // 获取数量
    
        snprintf(msg_data, sizeof(msg_data), "{ \"type\": \"uploadRoute\", \"data\" : [");
         //解析yaml文件
        for (int i = 0; i < count; ++i)
        {
            geometry_msgs::msg::Pose pose;
            std::string pose_key = "pose_" + std::to_string(i + 1); 
           // std::cout<<"pose_key ="<<pose_key<<std::endl;
            pose.position.x = config["poses"][i][pose_key]["position_x"].as<double>();
            pose.position.y = config["poses"][i][pose_key]["position_y"].as<double>();
            pose.position.z = config["poses"][i][pose_key]["position_z"].as<double>();
            pose.orientation.x = config["poses"][i][pose_key]["orientation_x"].as<double>();
            pose.orientation.y = config["poses"][i][pose_key]["orientation_y"].as<double>();
            pose.orientation.z = config["poses"][i][pose_key]["orientation_z"].as<double>();
            pose.orientation.w = config["poses"][i][pose_key]["orientation_w"].as<double>();
            //存入map
            poses_map.insert({std::to_string(i), pose});
            //上报web服务器
            
            std::string str_count=std::to_string(i);
            std::string poseName = "pose" + std::to_string(i + 1);
            snprintf(temp_data, sizeof(temp_data), "{\"route\":\"%s\",\"routeName\":\"%s\"}", str_count.c_str(), poseName.c_str());
            if (i < count - 1) {          
                snprintf(msg_data + strlen(msg_data), sizeof(msg_data) - strlen(msg_data), "%s,", temp_data);
            } else {
                snprintf(msg_data + strlen(msg_data), sizeof(msg_data) - strlen(msg_data), "%s", temp_data);
            }

        }
        snprintf(msg_data + strlen(msg_data), sizeof(msg_data) - strlen(msg_data), "]}");
       // std::cout<<msg_data<<std::endl;

        auto uploadRoute = std_msgs::msg::String();
        uploadRoute.data = msg_data;  // 将C字符串赋值给消息的data成员
        ws_publisher_->publish(uploadRoute);



        geometry_msgs::msg::Pose pose;
        std::string returnPoint_key = "returnPoint"; 
        pose.position.x = config["returnPoint"]["position_x"].as<double>();
        pose.position.y = config["returnPoint"]["position_y"].as<double>();
        pose.position.z = config["returnPoint"]["position_z"].as<double>();
        pose.orientation.x = config["returnPoint"]["orientation_x"].as<double>();
        pose.orientation.y = config["returnPoint"]["orientation_y"].as<double>();
        pose.orientation.z = config["returnPoint"]["orientation_z"].as<double>();
        pose.orientation.w = config["returnPoint"]["orientation_w"].as<double>();
        poses_map.insert({"returnPoint", pose});
        //char msg_data[256];
        snprintf(msg_data, sizeof(msg_data),
                "{ \"type\": \"uploadReturnPoint\", \"data\" : [{\"returnPoint\":\"%s\",\"returnPointName\":\"%s\"}]}",
        "returnPoint",
        "返回点");
       // std::cout<<msg_data<<std::endl;
       
        auto uploadReturnPoint = std_msgs::msg::String();
        uploadReturnPoint.data = msg_data;  // 将C字符串赋值给消息的data成员
        ws_publisher_->publish(uploadReturnPoint);


        for (const auto& entry : poses_map)
    {
        std::cout << "Pose Key: " << entry.first << std::endl;
        const geometry_msgs::msg::Pose& pose = entry.second;

        // 输出位置信息
        std::cout << "  Position: ("
                  << pose.position.x << ", "
                  << pose.position.y << ", "
                  << pose.position.z << ")" << std::endl;

        // 输出方向信息
        std::cout << "  Orientation: ("
                  << pose.orientation.x << ", "
                  << pose.orientation.y << ", "
                  << pose.orientation.z << ", "
                  << pose.orientation.w << ")" << std::endl;
    }

    }

private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ws_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hw_work_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr hw_mode_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planning_goal_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ws_subscription_;

    rclcpp::Subscription<mycan_msgs::msg::Hwcan>::SharedPtr hwcan_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_cmd;
    rclcpp::Subscription<tier4_planning_msgs::msg::RouteState>::SharedPtr autoware_mode_cmd;



    net::io_context io_context_;
    websocket::stream<tcp::socket> ws_;


    std::atomic<bool> shutdown_requested_;
    std::mutex ws_mutex_;
    std::thread read_thread_;
    std::thread io_context_thread_;
    net::executor_work_guard<net::io_context::executor_type> work_guard_;
    mycan_msgs::msg::Hwcan hw_msg;
    std::string Vin="LCFCHBHE1P1010046";
    std::string car_no="京N GN0000";
    std::string location="北京";
    std::string params="string";
    std::string remark="备注";
    int autoware_mode_msg;
    int offlineVideoReq;
    std::string rtmp_msg;
    std::map<std::string, geometry_msgs::msg::Pose> poses_map;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebSocketNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
