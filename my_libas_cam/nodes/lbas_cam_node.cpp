#include "rclcpp/rclcpp.hpp"
#include "my_libas_cam/lbas_cam.h"
#include <sensor_msgs/image_encodings.hpp>
#include <sstream>
#include <std_srvs/srv/empty.h>
#include <iostream>

int main(int argc, char **argv)
{
	std::string camera_name;
	float camera_rate=0.0;
	lbas_cam::LBASCam cam_;
	sensor_msgs::msg::Image			image_msg;
	sensor_msgs::msg::CameraInfo 	cam_msg;
	rclcpp::init(argc, argv);


	rclcpp::NodeOptions lbas_camera;
	rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
	qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
	qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	//auto qos =rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
	auto node = std::make_shared<rclcpp::Node>("my_libas_cam");
	image_transport::Publisher image_pub;

    	image_transport::ImageTransport it(node); 
    	
	//cam_.Cam_info();

	if(!cam_.Cam_info(lbas_camera,node))

	{
		return false;
	}
	if(!cam_.Cam_init())

	{
		return false;
	}
	camera_name=cam_.Cam_GetID();	
	//image_transport::CameraPublisher pub = it.advertiseCamera("/"+camera_name+"/image_raw", 1); 
	
	//image_transport::CameraPublisher pub = it.advertiseCamera("/sensing/camera/camera0/image_rect_color", ); 
	auto pub_qos1=node->create_publisher<sensor_msgs::msg::Image>("/sensing/camera/camera0/image_rect_color",qos_profile);
	auto pub_qos2=node->create_publisher<sensor_msgs::msg::CameraInfo>("/sensing/camera/camera0/camera_info",qos_profile);
	//topic·¢²¼ÆµÂÊ
	//-------------------10hz-----------------//
	camera_rate=cam_.Cam_FrameRate();
	rclcpp::Rate rate((int)camera_rate);
	//std::cout<<"Start to Grab,Refresh:"<<(int)camera_rate<<std::endl;qos_profile
	while (rclcpp::ok())
	{
		if(!cam_.GrabImage(&image_msg,&cam_msg,node))
		{
			std::cout<<"continue"<<std::endl;
			continue;
		}
		//pub.publish(image_msg, cam_msg); 
		//auto now = node->now();
		//cam_msg.header.stamp 		= 	now;	
		pub_qos1->publish(image_msg);
		pub_qos2->publish(cam_msg);
		//std::cout<<"publish"<<std::endl;
		//std::cout<<"Start to Grab,Refresh:"<<(int)camera_rate<<std::endl;
		rate.sleep();
		
	 
	}

	return true;
}
