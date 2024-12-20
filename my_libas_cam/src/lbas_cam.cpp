#include <my_libas_cam/lbas_cam.h>
#include <string>

namespace lbas_cam 
{
	std::string camera_name;
	LBASCam::CAM_INFO *cam_info = new LBASCam::CAM_INFO; 
	//memset(cam_info, 0, sizeof(LBASCam::CAM_INFO)); 
	bool IsColor(MvGvspPixelType enType)
	{
		switch(enType)
		{
			case PixelType_Gvsp_RGB8_Packed:
			case PixelType_Gvsp_YUV422_Packed:
			case PixelType_Gvsp_YUV422_YUYV_Packed:
			case PixelType_Gvsp_BayerGR8:
			case PixelType_Gvsp_BayerRG8:
			case PixelType_Gvsp_BayerGB8:
			case PixelType_Gvsp_BayerBG8:
			case PixelType_Gvsp_BayerGB10:
			case PixelType_Gvsp_BayerGB10_Packed:
			case PixelType_Gvsp_BayerBG10:
			case PixelType_Gvsp_BayerBG10_Packed:
			case PixelType_Gvsp_BayerRG10:
			case PixelType_Gvsp_BayerRG10_Packed:
			case PixelType_Gvsp_BayerGR10:
			case PixelType_Gvsp_BayerGR10_Packed:
			case PixelType_Gvsp_BayerGB12:
			case PixelType_Gvsp_BayerGB12_Packed:
			case PixelType_Gvsp_BayerBG12:
			case PixelType_Gvsp_BayerBG12_Packed:
			case PixelType_Gvsp_BayerRG12:
			case PixelType_Gvsp_BayerRG12_Packed:
			case PixelType_Gvsp_BayerGR12:
			case PixelType_Gvsp_BayerGR12_Packed:
				return true;
			default:
				return false;
		}
	}
	bool IsMono(MvGvspPixelType enType)
	{
		switch(enType)
		{
			case PixelType_Gvsp_Mono10:
			case PixelType_Gvsp_Mono10_Packed:
			case PixelType_Gvsp_Mono12:
			case PixelType_Gvsp_Mono12_Packed:
				return true;
			default:
				return false;
		}
	}

	void LBASCam::shutdown()
	{
		int nRet = MV_OK;
		// 停止取流
		// end grab image
		nRet = MV_CC_StopGrabbing(handle_);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_StopGrabbing fail! nRet [%x]", nRet);
			return;
		}

		// 关闭设备
		// close device
		nRet = MV_CC_CloseDevice(handle_);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_CloseDevice fail! nRet [%x]", nRet);
			return;
		}

		// 销毁句柄	
		// destroy handle
		nRet = MV_CC_DestroyHandle(handle_);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_DestroyHandle fail! nRet [%x]", nRet);
			return;
		}
	} 

	bool LBASCam::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) 
	{
		if (NULL == pstMVDevInfo) {
			//RCLCPP_INFO("The Pointer of pstMVDevInfo is NULL!");
			return false;
		}
		if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
		{
			int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
			int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
			int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
			int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
			// ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
			//RCLCPP_INFO("Device Model Name: %s", 	pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
			//RCLCPP_INFO("CurrentIp: %d.%d.%d.%d" , nIp1, nIp2, nIp3, nIp4);
			//RCLCPP_INFO("UserDefinedName: %s" , 	pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
			//RCLCPP_INFO("Serial Number: %s", 		pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
		} else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
		{
			//RCLCPP_INFO("Device Model Name: %s", 	pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
			//RCLCPP_INFO("UserDefinedName: %s", 	pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
			//RCLCPP_INFO("Serial Number: %s", 		pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
		} else 
		{
			//RCLCPP_INFO("Not support.");
		}

		return true;
	}
	std::string LBASCam:: Cam_GetID() 
	{
        return cam_info->DeviceUserID;
    }
	float LBASCam:: Cam_FrameRate() 
	{
        return cam_info->AcquisitionFrameRate;
    }

	bool LBASCam::Cam_info(const rclcpp::NodeOptions &node_options, std::shared_ptr<rclcpp::Node> &node)
	{
		//从yaml上面获取值，如果获取不到，就用当前值代替
		//src/lbascam/config/camera_0.yaml
		//src/lbascam/config/camera_name.yaml
		// node_->param("Width", 						cam_info->Width, 						3072);
		// node_->param("Height", 						cam_info->Height, 						2048);
		// node_->param("OffsetX", 						cam_info->OffsetX,						0);
		// node_->param("OffsetY", 						cam_info->OffsetY,						0);
		// node_->param("AcquisitionFrameRateEnable", 	cam_info->AcquisitionFrameRateEnable, 	false);
		// node_->param("AcquisitionFrameRate", 		cam_info->AcquisitionFrameRate,			(float)10);
		// node_->param("BurstFrameCount", 				cam_info->BurstFrameCount, 				1); // 一次触发采集的次数
		// node_->param("ExposureAuto", 				cam_info->ExposureAuto, 				0);
		// node_->param("ExposureUpperLimit",			cam_info->ExposureUpperLimit, 			20000);
		// node_->param("ExposureLowerLimit", 			cam_info->ExposureLowerLimit, 			15);
		// node_->param("ExposureTime", 				cam_info->ExposureTime,					(float)50000);
		// node_->param("GammaEnable", 					cam_info->GammaEnable, 					false);
		// node_->param("Gamma", 						cam_info->Gamma, 						(float)0.7);
		// node_->param("GainAuto", 					cam_info->GainAuto, 					2);
		// node_->param("Gain", 						cam_info->Gain, 						(float)0.0);
		// node_->param("DigitalShiftEnable", 			cam_info->DigitalShiftEnable, 			false);
		// node_->param("DigitalShift", 				cam_info->DigitalShift, 				(float)0.5);
		// node_->param("TriggerMode", 					cam_info->TriggerMode, 					0); //1
		// node_->param("TriggerSource", 				cam_info->TriggerSource, 				2);
		// node_->param("TriggerActivation", 			cam_info->TriggerActivation, 			0);
		// node_->param("TriggerDelay", 				cam_info->TriggerDelay, 				(float)0.0);
		// node_->param("TriggerCacheEnable", 			cam_info->TriggerCacheEnable, 			true);
		// node_->param("LineSelector", 				cam_info->LineSelector, 				1);
		// node_->param("LineDebouncerTime", 			cam_info->LineDebouncerTime, 			50);
		// //Image control
		// node_->param("PixelFormat", 					cam_info->PixelFormat, 					(std::string)"BayerRG8");
		// node_->param("RosEncoding", 					cam_info->RosEncoding, 					(std::string)"bayer_rggb8");
		// node_->param("BinningSelector", 				cam_info->BinningSelector, 				0);
		// node_->param("BinningHorizontal", 			cam_info->BinningHorizontal, 			1);
		// node_->param("BinningVertical", 				cam_info->BinningVertical, 				1);
		// //cam color 
		// node_->param("SaturationEnable", 			cam_info->SaturationEnable, 			true);
		// node_->param("Saturation", 					cam_info->Saturation, 					128);
		// node_->param("BalanceWhiteAuto", 			cam_info->BalanceWhiteAuto, 			0);
		// node_->param("BalanceWhiteAuto", 			cam_info->BalanceRatio_R, 				1100);
		// node_->param("BalanceWhiteAuto", 			cam_info->BalanceRatio_G, 				1024);
		// node_->param("BalanceWhiteAuto", 			cam_info->BalanceRatio_B, 				1826);
		// //cam id
		// node_->param("DeviceUserID", 				cam_info->DeviceUserID, 				(std::string)"camera_0");
		// node_->param("DeviceSerialNumber", 			cam_info->DeviceSerialNumber, 			(std::string)"123456789");
		// node_->param("CamType", 						cam_info->CamType, 						0);
		//Gige Camera


		node->declare_parameter<int>("Width", 3072);
        node->get_parameter("Width", cam_info->Width);
        node->declare_parameter<int>("Height", 2048);
        node->get_parameter("Height", cam_info->Height);
        node->declare_parameter<int>("OffsetX", 0);
        node->get_parameter("OffsetX", cam_info->OffsetX);
        node->declare_parameter<int>("OffsetY", 0);
        node->get_parameter("OffsetY", cam_info->OffsetY);
        node->declare_parameter<bool>("AcquisitionFrameRateEnable", false);
        node->get_parameter("AcquisitionFrameRateEnable", cam_info->AcquisitionFrameRateEnable);
        node->declare_parameter<float>("AcquisitionFrameRate", 10.00);
        node->get_parameter("AcquisitionFrameRate", cam_info->AcquisitionFrameRate);
        node->declare_parameter<int>("BurstFrameCount", 1);
        node->get_parameter("BurstFrameCount", cam_info->BurstFrameCount);
        node->declare_parameter<int>("ExposureAuto", 1);
        node->get_parameter("ExposureAuto", cam_info->ExposureAuto);
        node->declare_parameter<int>("ExposureUpperLimit", 20000);
        node->get_parameter("ExposureUpperLimit", cam_info->ExposureUpperLimit);
        node->declare_parameter<int>("ExposureLowerLimit", 15);
        node->get_parameter("ExposureLowerLimit", cam_info->ExposureLowerLimit);
        node->declare_parameter<float>("ExposureTime", 50000);
        node->get_parameter("ExposureTime", cam_info->ExposureTime);
        node->declare_parameter<bool>("GammaEnable", false);
        node->get_parameter("GammaEnable", cam_info->GammaEnable);
        node->declare_parameter<float>("Gamma", 0.7);
        node->get_parameter("Gamma", cam_info->Gamma);
        node->declare_parameter<int>("GainAuto", 2);
        node->get_parameter("GainAuto", cam_info->GainAuto);
        node->declare_parameter<float>("Gain", 0.0);
        node->get_parameter("Gain", cam_info->Gain);
        node->declare_parameter<bool>("DigitalShiftEnable", false);
        node->get_parameter("DigitalShiftEnable", cam_info->DigitalShiftEnable);
        node->declare_parameter<float>("DigitalShift", 0.5);
        node->get_parameter("DigitalShift", cam_info->DigitalShift);
        node->declare_parameter<int>("TriggerMode", 0);
        node->get_parameter("TriggerMode", cam_info->TriggerMode);
      	node->declare_parameter<int>("TriggerSource", 2);
        node->get_parameter("TriggerSource", cam_info->TriggerSource);
        node->declare_parameter<int>("TriggerActivation", 0);
        node->get_parameter("TriggerActivation", cam_info->TriggerActivation);
        node->declare_parameter<float>("TriggerDelay", 0);
        node->get_parameter("TriggerDelay", cam_info->TriggerDelay);
        node->declare_parameter<bool>("TriggerCacheEnable", true);
        node->get_parameter("TriggerCacheEnable", cam_info->TriggerCacheEnable);
        node->declare_parameter<int>("LineSelector", 1);
        node->get_parameter("LineSelector", cam_info->LineSelector);
        node->declare_parameter<int>("LineDebouncerTime", 50);
        node->get_parameter("LineDebouncerTime", cam_info->LineDebouncerTime);
        node->declare_parameter<std::string>("PixelFormat","BayerRG8");
        node->get_parameter("PixelFormat", cam_info->PixelFormat);
        node->declare_parameter<std::string>("RosEncoding","bayer_rggb8");
        node->get_parameter("RosEncoding", cam_info->RosEncoding);
        node->declare_parameter<int>("BinningSelector", 0);
        node->get_parameter("BinningSelector", cam_info->BinningSelector);
        node->declare_parameter<int>("BinningHorizontal", 1);
        node->get_parameter("BinningHorizontal", cam_info->BinningHorizontal);
        node->declare_parameter<int>("BinningVertical", 1);
        node->get_parameter("BinningVertical", cam_info->BinningVertical);
        node->declare_parameter<bool>("SaturationEnable", true);
        node->get_parameter("SaturationEnable", cam_info->SaturationEnable);
        node->declare_parameter<int>("Saturation", 128);
        node->get_parameter("Saturation", cam_info->Saturation);
        node->declare_parameter<int>("BalanceWhiteAuto", 0);
        node->get_parameter("BalanceWhiteAuto", cam_info->BalanceWhiteAuto);
        node->declare_parameter<int>("BalanceRatio_R", 1100);
        node->get_parameter("BalanceRatio_R", cam_info->BalanceRatio_R);
        node->declare_parameter<int>("BalanceRatio_G", 1024);
        node->get_parameter("BalanceRatio_G", cam_info->BalanceRatio_G);
        node->declare_parameter<int>("BalanceRatio_B", 1826);
        node->get_parameter("BalanceRatio_B", cam_info->BalanceRatio_B);
        node->declare_parameter<std::string>("DeviceUserID","camera_0");
        node->get_parameter("DeviceUserID", cam_info->DeviceUserID);
        node->declare_parameter<std::string>("DeviceSerialNumber","123456789");
        node->get_parameter("DeviceSerialNumber", cam_info->DeviceSerialNumber);
        node->declare_parameter<int>("CamType", 0);
        node->get_parameter("CamType", cam_info->CamType);

		return true;
	}

	 bool LBASCam::Cam_info()
	{
		//从yaml上面获取值，如果获取不到，就用当前值代替
		//src/lbascam/config/camera_0.yaml
		//src/lbascam/config/camera_name.yaml
		cam_info->Width=720;
		cam_info->Height=480;
		cam_info->OffsetX=0;
		cam_info->OffsetY=0;
		cam_info->AcquisitionFrameRateEnable=false;
		cam_info->AcquisitionFrameRate=41;
		cam_info->BurstFrameCount=1; // 一次触发采集的次数
		cam_info->ExposureAuto=0;
		cam_info->ExposureUpperLimit=20000;
		cam_info->ExposureLowerLimit=15;
		cam_info->ExposureTime=50000;
		cam_info->GammaEnable=false;
		cam_info->Gamma=0.7;
		cam_info->GainAuto=2;
		cam_info->Gain=0.0;
		cam_info->DigitalShiftEnable=false;
		cam_info->DigitalShift=0.5;
		cam_info->TriggerMode=0; //1
		cam_info->TriggerSource=2;
		cam_info->TriggerActivation=0;
		cam_info->TriggerDelay=0.0;
		cam_info->TriggerCacheEnable=true;
		cam_info->LineSelector=1;
		cam_info->LineDebouncerTime=50;
		//Image control
		cam_info->PixelFormat="BayerRG8";
		cam_info->RosEncoding="rgb8";
		cam_info->BinningSelector=0;
		cam_info->BinningHorizontal=1;
		cam_info->BinningVertical=1;
		//cam color 
		cam_info->SaturationEnable=true;
		cam_info->Saturation=128;
		cam_info->BalanceWhiteAuto=0;
		cam_info->BalanceRatio_R=1100;
		cam_info->BalanceRatio_G=1024;
		cam_info->BalanceRatio_B=1826;
		//cam id
		cam_info->DeviceUserID="camera_0";
		cam_info->DeviceSerialNumber="DA2942350";
		cam_info->CamType=1;
		//Gige Camera
		return true;
	}

	bool LBASCam::Cam_init()
	{
		int nRet = MV_OK;

		// 枚举设备
		// enum device
		MV_CC_DEVICE_INFO_LIST stDeviceList;
		nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_EnumDevices fail! nRet [%x]", nRet);
			std::cout<<"MV_CC_EnumDevices fail! nRet  "<<nRet<<std::endl;
			return false;
		}
		unsigned int nIndex = 0;	
		if (stDeviceList.nDeviceNum > 0) 
		{
			for (int i = 0; i < stDeviceList.nDeviceNum; i++) 
			{
					//RCLCPP_INFO("[device %d]:", i);
					MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
					if (NULL == pDeviceInfo){
						break;
					} 
					if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
					{
						if(strcmp(cam_info->DeviceSerialNumber.c_str(), (char *)pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber) == 0) 
						{
							nIndex = i;
						} 
					}
					else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE)
					{
						if (strcmp(cam_info->DeviceSerialNumber.c_str(),(const char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber)==0)
						{
							nIndex = i;

						}
					}
					PrintDeviceInfo(pDeviceInfo);            
			}
		}else
		{
			//RCLCPP_ERROR("Find No Devices!");
			std::cout<<"Find No Devices!"<<std::endl;
			return false;
		}
		// select device and create handle
		nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[nIndex]);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_CreateHandle fail! nRet [%x]", nRet);
			std::cout<<"MV_CC_CreateHandle fail nRet"<<nRet<<std::endl;
			return false;
		}
		nRet = MV_CC_OpenDevice(handle_);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_OpenDevice fail! nRet [%x]", nRet);
			std::cout<<"MV_CC_OpenDevice fail!"<<std::endl;
			return false;
		}else
		{
			//RCLCPP_INFO("OpenDevice:%s!",cam_info->DeviceSerialNumber.c_str());	
		}
		// ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
		if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) 
		{
			int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
			if (nPacketSize > 0) 
			{
				nRet = MV_CC_SetIntValue(handle_,"GevSCPSPacketSize",nPacketSize);
				if(nRet != MV_OK) 
				{
					//RCLCPP_WARN("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
					std::cout<<"Warning: Set Packet Size fail nRet"<<std::endl;				};

			} else 
			{
				//RCLCPP_WARN("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
				std::cout<<"Warning: Get Packet Size fail nRet"<<std::endl;
			}
		}
		// 设置触发模式为off
		// set trigger mode as off
		nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", cam_info->TriggerMode);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_SetTriggerMode fail! nRet [%x]", nRet);
			std::cout<<"MV_CC_SetTriggerMode fail"<<std::endl;	
			return false;
		}
		 //设置图像大小
		nRet = MV_CC_SetIntValueEx(handle_, "Width", cam_info->Width);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_Set Width fail! nRet [%x]", nRet);
			std::cout<<"MV_CC_Set Width fail  nRet "<<std::hex<<nRet<<std::endl;
			return false;
		}
		nRet = MV_CC_SetIntValueEx(handle_, "Height", cam_info->Height);
		if (MV_OK != nRet)
		{
			//RCLCPP_ERROR("MV_CC_Set Height fail! nRet [%x]", nRet);
			std::cout<<"MV_CC_Set Height fail"<<std::endl;
			return false;
		}
		#if 0
		//先注释，有需要的自行移出
		nRet = MV_CC_SetIntValue(handle_,"OffsetX",cam_info->OffsetX);
		if (MV_OK != nRet)
		{
			ROS_ERROR("MV_CC_Set OffsetX fail! nRet [%x]", nRet);

			return false;
		}
		nRet = MV_CC_SetIntValue(handle_,"OffsetY",cam_info->OffsetY);
		if (MV_OK != nRet)
		{
			ROS_ERROR("MV_CC_Set OffsetY fail! nRet [%x]", nRet);
			return false;
		}
		#endif
		
		nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", cam_info->ExposureTime);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_Set ExposureTime fail! nRet [%x]", nRet);
			std::cout<<"MV_CC_Set ExposureTime fail"<<std::endl;
			return false;
		}
		#if 0
		//先注释，有需要的自行移出
		//设置自动，如果使用自动曝光，ExposureTime就不需要再设置了
		nRet = MV_CC_SetEnumValue(handle_, "ExposureMode",0);//0：Timed
		nRet = MV_CC_SetFloatValue(handle_, "ExposureAuto", cam_info->ExposureAuto);//0：off 1：once 2：Continuous
		if (MV_OK != nRet)
		{
			ROS_ERROR("Set ExposureAuto fail nRet [0xd%]\n!", nRet);
			return false;
		}else{
			//只有自动曝光或者自动增益开启后，Brightness亮度值方可设置
			nRet = MV_CC_SetIntValue(handle_,"Brightness",160);//根据需要设置
			if(nRet != MV_OK)
			{
				ROS_ERROR("Set BrightnessnRet [0x%x]!", nRet);
				return false;
			}
		}
		nRet = MV_CC_SetIntValue(handle_,"ExposureUpperLimit",cam_info->ExposureUpperLimit);
		if (MV_OK != nRet)
		{
			ROS_ERROR("MV_CC_Set ExposureUpperLimit fail! nRet [%x]", nRet);
			return false;
		}
		nRet = MV_CC_SetIntValue(handle_,"ExposureLowerLimit",cam_info->ExposureLowerLimit);
		if (MV_OK != nRet)
		{
			ROS_ERROR("MV_CC_Set ExposureLowerLimit fail! nRet [%x]", nRet);
			return false;
		}
		#endif
		//彩色相机，需要根据相机实际支持的图像格式设置
		nRet = MV_CC_SetEnumValue(handle_, "PixelFormat",color_codes[cam_info->PixelFormat]);
		if (MV_OK != nRet)
		{
			//RCLCPP_ERROR("Set PixelFormat :%s fail! nRet [0x%x]\n", cam_info->PixelFormat.c_str(),nRet);
			std::cout<<"Set PixelFormat"<<std::endl;
			return false;
		}
		
		// MVCC_FLOATVALUE Cam_Rate;;
		// memset(&Cam_Rate, 0, sizeof(MVCC_FLOATVALUE));
		// nRet = MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &Cam_Rate);
		// if (MV_OK != nRet)
		// {
		// 	//RCLCPP_ERROR("Get AcquisitionFrameRate fail! nRet [0x%x]\n", nRet);
		// 	std::cout<<"Get AcquisitionFrameRate fail"<<std::endl;
		// 	return nRet;
		// }
		//cam_info->AcquisitionFrameRate=Cam_Rate.fCurValue;
		
		#if 0
		//先注释，有需要的自行移出
		nRet = MV_CC_SetIntValue(handle_,"AcquisitionBurstFrameCount",cam_info->AcquisitionBurstFrameCount);
		if (MV_OK != nRet)
		{
			ROS_ERROR("MV_CC_Set AcquisitionBurstFrameCount fail! nRet [%x]", nRet);
			return false;
		}
		nRet = MV_CC_SetBoolValue(handle_, "GammaEnable", cam_info->GammaEnable);
		if (MV_OK != nRet)
		{
			ROS_ERROR("MV_CC_Set GammaEnable fail! nRet [%x]", nRet);
			return false;
		}
		nRet = MV_CC_SetFloatValue(handle_, "Gamma",cam_info->Gamma);
		if (MV_OK != nRet)
		{
			ROS_ERROR("Set Gamma failed! nRet [0xd%]\n", nRet);
			return false;
		}
		//设置手动白平衡
		if(cam_info->CamType)//彩色相机才需要
		{
			nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
			if (MV_OK != nRet)
			{
				ROS_ERROR("Set BalanceWhiteAuto failed! nRet [0xd%]\n", nRet);
				return false;
			}
			int BalanceRatio_Value[3]={};//R、G、B
			//需要LBAS调试效果后，读出设置
			BalanceRatio_Value[0]=cam_info->BalanceRatio_R;
			BalanceRatio_Value[1]=cam_info->BalanceRatio_R;
			BalanceRatio_Value[2]=cam_info->BalanceRatio_R;
			for (int i = 0; i < 3; i++)
			{
				nRet = MV_CC_SetEnumValue(handle_, "BalanceRatioSelector", i);
				nRet = MV_CC_SetIntValue (handle_, "BalanceRatio",BalanceRatio_Value[i]);
				if (MV_OK != nRet)
				{
					ROS_ERROR("Set BalanceRatio failed! nRet [0xd%]\n", nRet);
					return false;
				}
			}
			
		}
		#endif
		// 开始取流		// start grab image
		
		nRet = MV_CC_StartGrabbing(handle_);
		if (MV_OK != nRet) 
		{
			//RCLCPP_ERROR("MV_CC_StartGrabbing fail! nRet [%x]", nRet);
			std::cout<<"MV_CC_StartGrabbing fail"<<std::endl;
			return false;
		}
		return true;
	}

	bool LBASCam::GrabImage(sensor_msgs::msg::Image* msg,sensor_msgs::msg::CameraInfo *cam_msg,std::shared_ptr<rclcpp::Node> &node) 
	{
		int nRet =0;
		MV_FRAME_OUT stImageInfo = {0};
		unsigned char *pConvertData = NULL;
		unsigned int nConvertDataSize = 0;
		rclcpp::Time DevTimeImageStamp;


	
		auto header = std_msgs::msg::Header();
		auto now = node->now();

		nRet = MV_CC_GetImageBuffer(handle_, &stImageInfo, 100);
		if (nRet == MV_OK) 
		{
			//ROS节点中传递图像数据-----------------------------------------------
			char ImageID[64]={0};
			sprintf(ImageID,"%d",stImageInfo.stFrameInfo.nFrameNum);
			 cam_msg->height = stImageInfo.stFrameInfo.nHeight; 
			 cam_msg->width = stImageInfo.stFrameInfo.nWidth; 
			// DevTimeImageStamp.sec  = stImageInfo.stFrameInfo.nDevTimeStampHigh; 
			// DevTimeImageStamp.nsec = rclcpp::Time(stImageInfo.stFrameInfo.nDevTimeStampHigh, stImageInfo.stFrameInfo.nDevTimeStampLow);		
			cam_msg->header.stamp 		= 	now;					//相机时间
			cam_msg->header.frame_id 	= 	ImageID;							//正常的使用，这个似乎不是传frame_num,如果需要使用这个字节，自行更改string
			msg->header.stamp 	  		= 	now;					//拿到图的时间
			msg->header.frame_id 		= 	ImageID;
			//ROS节点中传递图像数据-----------------------------------------------
			int64_t DevStamp=stImageInfo.stFrameInfo.nDevTimeStampHigh;
			DevStamp= (DevStamp << 32) + stImageInfo.stFrameInfo.nDevTimeStampLow;			
			//RCLCPP_INFO("%s:GetImage:%d,Width[%d],Height[%d],DevStamp:[%ld]", cam_info->DeviceUserID.c_str(),stImageInfo.stFrameInfo.nFrameNum,stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nHeight,DevStamp);
			
			MvGvspPixelType enDstPixelType = PixelType_Gvsp_Undefined;
			unsigned int nChannelNum = 0;
			//如果是彩色则转成RGB8
			if (IsColor(stImageInfo.stFrameInfo.enPixelType))
			{
				nChannelNum = 3;
				enDstPixelType = PixelType_Gvsp_BGR8_Packed;
			}
			//如果是黑白则转换成Mono8
			else if (IsMono(stImageInfo.stFrameInfo.enPixelType))
			{
				nChannelNum = 1;
				enDstPixelType = PixelType_Gvsp_Mono8;
			}
			else 
			{
				//ROS_INFO("Don't need to convert!\n");
				if(cam_info->CamType)
				{
					//color BGR
					msg->encoding = sensor_msgs::image_encodings::BGR8; 
					//fillImage是耗时操作
					sensor_msgs::fillImage(*msg, "bgr8", stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nWidth*3, stImageInfo.pBufAddr);
						
				}else
				{	
					//mono8
					msg->encoding = sensor_msgs::image_encodings::MONO8; 
					sensor_msgs::fillImage(*msg, "mono8", stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nWidth, stImageInfo.pBufAddr);	
				}
				
			}
			if (enDstPixelType != PixelType_Gvsp_Undefined)
			{

				pConvertData = (unsigned char*)malloc(stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight * nChannelNum);
				if (NULL == pConvertData)
				{
					//RCLCPP_ERROR("malloc pConvertData fail!\n");
					std::cout<<"malloc pConvertData fail"<<std::endl;

					nRet = MV_E_RESOURCE;
					return false;
				}
				nConvertDataSize = stImageInfo.stFrameInfo.nWidth * stImageInfo.stFrameInfo.nHeight * nChannelNum;
				// ch:像素格式转换 | en:Convert pixel format 
				MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
				stConvertParam.nWidth = stImageInfo.stFrameInfo.nWidth;                 //ch:图像width| en:image width
				stConvertParam.nHeight = stImageInfo.stFrameInfo.nHeight;               //ch:图像height| en:image height
				stConvertParam.pSrcData = stImageInfo.pBufAddr;                         //ch:输入数据缓存 | en:input data buffer
				stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLen;         //ch:输入数据大小 | en:input data size
				stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format
				stConvertParam.enDstPixelType = enDstPixelType;                         //ch:输出像素格式 | en:output pixel format
				stConvertParam.pDstBuffer = pConvertData;                               //ch:输出数据缓存 | en:output data buffer
				stConvertParam.nDstBufferSize = nConvertDataSize;                       //ch:输出缓存大小 | en:output buffer size
				nRet = MV_CC_ConvertPixelType(handle_, &stConvertParam);
				if (MV_OK != nRet)
				{
					//RCLCPP_ERROR("Convert Pixel Type fail! nRet [0x%x]\n", nRet);
						std::cout<<"Convert Pixel Type fail! nRet  "<<nRet<<std::endl;
					return false;
				}	
				if(enDstPixelType==PixelType_Gvsp_Mono8)
				{
					msg->encoding = sensor_msgs::image_encodings::MONO8; 
					sensor_msgs::fillImage(*msg, "mono8", stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nWidth, stConvertParam.pDstBuffer);	
				}
				else if(enDstPixelType==PixelType_Gvsp_BGR8_Packed)
				{
					msg->encoding = sensor_msgs::image_encodings::BGR8;
					sensor_msgs::fillImage(*msg, "bgr8", stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, stImageInfo.stFrameInfo.nWidth*3, stConvertParam.pDstBuffer);
				}
			}
			nRet = MV_CC_FreeImageBuffer(handle_, &stImageInfo);
			if (pConvertData) 
			{
				free(pConvertData);
			}
		} else
		{
			//RCLCPP_ERROR("No data[%x]\n", nRet);
			std::cout<<"No data "<<nRet<<std::endl;
			printf("  nRet=  %x\n", nRet);
			return false;
		}

		return true;
	}

}
