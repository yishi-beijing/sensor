/*********************************************************************
 *********************************************************************/
#ifndef LBAS_CAM_H
#define LBAS_CAM_H
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <iostream>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp> 
#include <sensor_msgs/fill_image.hpp>
#include "MvCameraControl.h"
#include "rclcpp/rclcpp.hpp"

namespace lbas_cam 
{

	class LBASCam 
	{
		public:
				 LBASCam(){};
				 ~LBASCam(){};
				 typedef struct _CAM_INFO_
				 {                    
					int			Width;				//图像宽
					int			Height;				//图像高
					int 		OffsetX;
					int 		OffsetY;
					bool 		AcquisitionFrameRateEnable;
					float 		AcquisitionFrameRate;
					int 		BurstFrameCount;
					int 		ExposureAuto;
					int 		ExposureUpperLimit;
					int 		ExposureLowerLimit;
					float 		ExposureTime;
					bool 		GammaEnable;
					float 		Gamma;
					int 		GainAuto;
					float		Gain;
					bool		DigitalShiftEnable;
					float		DigitalShift;
					int 		TriggerMode;
					int 		TriggerSource;
					int 		TriggerActivation;
					float 		TriggerDelay;
					bool 		TriggerCacheEnable;
					int 		LineSelector;
					int			LineDebouncerTime;
					std::string PixelFormat;
					std::string	RosEncoding;
					int			BinningSelector;
					int			BinningHorizontal;
					int			BinningVertical;
					//color camera
					bool 		SaturationEnable;
					int			Saturation;
					int			BalanceWhiteAuto;
					int			BalanceRatio_R;
					int			BalanceRatio_G;
					int			BalanceRatio_B;
					std::string DeviceUserID;
					std::string DeviceSerialNumber;
					int			CamType;
				 }CAM_INFO;
				bool Cam_info(const rclcpp::NodeOptions &node_options, std::shared_ptr<rclcpp::Node> &node);
				bool Cam_info();
				bool Cam_init();								//打开相机、并初始化相机参数
				std::string Cam_GetID();
				float Cam_FrameRate();				
				bool GrabImage(sensor_msgs::msg::Image* msg,sensor_msgs::msg::CameraInfo *cam_msg,std::shared_ptr<rclcpp::Node> &node);
				void shutdown();
		 private:
			bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);
			void* handle_ = NULL;
			std::map<std::string, int> color_codes
			{
				{"Mono8", 0x01080001},
				{"Mono10", 0x01100003},
				{"Mono12", 0x01100005},
				{"RGB8packed", 0x02180014},
				{"BGR8packed", 0x02180015},
				{"YUV422packed", 0x0210001F},
				{"YUV422_YUYV_packed", 0x02100032},
				
				{"BayerGR8", 	0x01080008},
				{"BayerRG8", 	0x01080009},
				{"BayerGB8", 	0x0108000A},
				{"BayerBG8", 	0x0108000B},
				{"BayerRBGG8", 	0x01080046},
				
				{"BayerGR10", 0x0110000C},
				{"BayerRG10", 0x0110000D},
				{"BayerGB10", 0x0110000E},
				{"BayerBG10", 0x0110000F},
				
				{"BayerGR10packed", 0x010C0026},
				{"BayerRG10packed", 0x010C0027},
				{"BayerGB10packed", 0x010C0028},
				{"BayerBG10packed", 0x010C0029},
				
				{"BayerGR12", 0x01100010},
				{"BayerRG12", 0x01100011},
				{"BayerGB12", 0x01100012},
				{"BayerBG12", 0x01100013},
				
				{"BayerGR12packed", 0x010C002A},
				{"BayerRG12packed", 0x010C002B},
				{"BayerGB12packed", 0x010C002C},
				{"BayerBG12packed", 0x010C002D}
			};

	};

}

#endif

