/*!
 * \file
 * \brief
 */

#ifndef CAMERANUI_HPP_
#define CAMERANUI_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler.hpp"

#include <Types/CameraInfo.hpp>

#include <opencv2/core/core.hpp>

#include <libfreenect_sync.h>

namespace Processors {
namespace CameraNUI {

class TypeTranslator {
public:
	static int fromStr(const std::string & s)
	{
		if (s == "FREENECT_VIDEO_RGB") return FREENECT_VIDEO_RGB;
		if (s == "FREENECT_VIDEO_BAYER") return FREENECT_VIDEO_BAYER;
		if (s == "FREENECT_VIDEO_IR_8BIT") return FREENECT_VIDEO_IR_8BIT;
        return FREENECT_VIDEO_RGB;
	}

	static std::string toStr(int t)
	{
		switch(t) {
			case FREENECT_VIDEO_RGB:     return "FREENECT_VIDEO_RGB";
			case FREENECT_VIDEO_BAYER:     return "FREENECT_VIDEO_BAYER";
			case FREENECT_VIDEO_IR_8BIT: return "FREENECT_VIDEO_IR_8BIT";
			default:             return "FREENECT_VIDEO_RGB";
		}
	}
};

class DepthTypeTranslator {
public:
	static int fromStr(const std::string & s)
	{
		if (s == "FREENECT_DEPTH_MM") return FREENECT_DEPTH_MM;
		if (s == "FREENECT_DEPTH_REGISTERED") return FREENECT_DEPTH_REGISTERED;
        return FREENECT_DEPTH_MM;
	}

	static std::string toStr(int t)
	{
		switch(t) {
			case FREENECT_DEPTH_MM:     return "FREENECT_DEPTH_MM";
			case FREENECT_DEPTH_REGISTERED:     return "FREENECT_DEPTH_REGISTERED";
			default: return "FREENECT_DEPTH_MM";
		}
	}
};

class ResolutionTranslator {
public:
	static int fromStr(const std::string & s)
	{
		if (s == "FREENECT_RESOLUTION_MEDIUM") return FREENECT_RESOLUTION_MEDIUM;
		if (s == "FREENECT_RESOLUTION_HIGH") return FREENECT_RESOLUTION_HIGH;
        return FREENECT_RESOLUTION_MEDIUM;
	}

	static std::string toStr(int t)
	{
		switch(t) {
			case FREENECT_RESOLUTION_MEDIUM: return "FREENECT_RESOLUTION_MEDIUM";
			case FREENECT_RESOLUTION_HIGH:   return "FREENECT_RESOLUTION_HIGH";
			default: return "FREENECT_RESOLUTION_MEDIUM";
		}
	}
};


/*!
 * \class CameraNUI
 * \brief CameraNUI processor class.
 */
class CameraNUI: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CameraNUI(const std::string & name = "");

	/*!
	 * Destructor
	 */
	virtual ~CameraNUI();

	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Retrieves data from device.
	 */
	bool onStep() { return true; };

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	/*!
	 * Event handler function.
	 */
	void onNextImage();
	void setAngle();
	void setAngleFromProperty();


	/// Event handler.
	Base::EventHandler <CameraNUI> h_onNextImage;

	Base::Property <bool> sync;
	Base::Property <int> index;
	Base::Property <int> angle;
	Base::Property <int, TypeTranslator> imageType;
	Base::Property <int, DepthTypeTranslator> depthType;
	Base::Property <int, ResolutionTranslator> resolution;

	Base::DataStreamIn <int> in_angle;

	Base::DataStreamIn <Base::UnitType> trigger;
	Base::DataStreamOut <cv::Mat> outImg;
	Base::DataStreamOut <cv::Mat> outDepthMap;
	Base::DataStreamOut <Types::CameraInfo> camera_info;

	cv::Mat cameraFrameVga;
	cv::Mat cameraFrameSxga;
	cv::Mat depthFrame;
	cv::Mat show;

	Types::CameraInfo m_camera_info;

	Base::Property<bool> triggered;
	
};

} //: namespace CameraNUI
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CameraNUI", Processors::CameraNUI::CameraNUI)

#endif /* CAMERANUI_HPP_ */
