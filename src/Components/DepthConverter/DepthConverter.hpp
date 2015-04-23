/*!
 * \file
 * \brief
 */

#ifndef DEPTHCONVERTER_HPP_
#define DEPTHCONVERTER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include "DepthModeTranslator.hpp"
#include <Types/CameraInfo.hpp>

#include <opencv2/core/core.hpp>

namespace Processors {
namespace DepthConverter {

/*!
 * \class DepthConverter
 * \brief DepthConverter processor class.
 */
class DepthConverter: public Base::Component
{
public:
	/*!
	 * Constructor.
	 */
	DepthConverter(const std::string & name = "");

	/*!
	 * Destructor
	 */
	virtual ~DepthConverter();

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
	bool onStep();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	/// Event handler.
	Base::EventHandler <DepthConverter> h_onNewDepth;

	/// Input data stream
	Base::DataStreamIn <cv::Mat> in_depth;

	/// Output data stream - processed image
	Base::DataStreamOut <cv::Mat> out_img;

	/// Input data stream - camera info required for transformation of depth to xyz coordinates
	Base::DataStreamIn<Types::CameraInfo> in_camera_info;

private:
	void convertToDisparityMap(cv::Mat& data, cv::Mat& dataOut);
	void convertToDisparityMap32f(cv::Mat& data, cv::Mat& dataOut);
	void convertToPointCloudMap(cv::Mat& data, cv::Mat& dataOut);
	void convertToValidPixelsMap(cv::Mat& data, cv::Mat& dataOut);
	void convertToRainbowMap(cv::Mat& data, cv::Mat& dataOut);


	void onNewDepth();

	// depth map normalization factor
	static const float SCALE_FACTOR = 255.0/2048.0;
	// openKinect documentation:
	// "When the Kinect can't see the ir reflection
	// or has no depth data for a pixel, it returns
	// 2047 for the depth value"
	static const int INVALID_PIXEL = 2047;
	static const int INVALID_COORDINATE = 100000;
	// baseline and focal length from opencv properties
	static const int BASELINE = 75;
	static const int FOCAL_LENGTH = 575;
	// for now only this resolution fully supported
//	static const int COLS = 640;
//	static const int ROWS = 480;

    Base::Property<DepthMode, DepthModeTranslator> depthMode;

	cv::Mat m_depth;
	cv::Mat m_out;
};

}//: namespace DepthConverter
}//: namespace Processors


/*
 * Register processor component.
 */
REGISTER_COMPONENT("DepthConverter", Processors::DepthConverter::DepthConverter)

#endif /* DEPTHCONVERTER_HPP_ */
