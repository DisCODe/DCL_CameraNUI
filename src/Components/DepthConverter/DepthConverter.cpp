/*!
 * \file
 * \brief
 */

#include <memory>
#include <string>

#include "DepthConverter.hpp"
#include "Common/Logger.hpp"

namespace Processors {
namespace DepthConverter {

DepthConverter::DepthConverter(const std::string & name) :
		Base::Component(name),
		depthMode("depth_mode",rawMap, "combo") {
	LOG(LTRACE) << "Hello DepthConverter\n";

    registerProperty(depthMode);
}

DepthConverter::~DepthConverter() {
	LOG(LTRACE) << "Good bye DepthConverter\n";
}

void DepthConverter::prepareInterface() {

	// Register data streams, events and event handlers HERE!
	registerStream("in_depth", &in_depth);
	registerStream("in_camera_info", &in_camera_info);

	registerStream("out_depth", &out_img);
	registerStream("out_img", &out_img);

	registerHandler("onNewDepth", boost::bind(&DepthConverter::onNewDepth, this));
	addDependency("onNewDepth", &in_depth);
}

bool DepthConverter::onInit() {
	LOG(LTRACE) << "DepthConverter::initialize\n";


	return true;
}

bool DepthConverter::onFinish() {
	LOG(LTRACE) << "DepthConverter::finish\n";

	return true;
}

bool DepthConverter::onStep() {
	LOG(LTRACE) << "DepthConverter::step\n";
	return true;
}

bool DepthConverter::onStop() {
	return true;
}

bool DepthConverter::onStart() {
	return true;
}

void DepthConverter::onNewDepth() {
	m_depth = in_depth.read();

	switch(depthMode) {
	case normalized:
		m_depth.convertTo( m_out, CV_8UC1, SCALE_FACTOR );
		break;
	case disparityMap:
		convertToDisparityMap(m_depth, m_out);
		break;
	case dM32f:
		convertToDisparityMap32f(m_depth, m_out);
		break;
	case pointCloud:
		convertToPointCloudMap(m_depth, m_out);
		break;
	case valid:
		convertToValidPixelsMap(m_depth, m_out);
		break;
	case rainbow:
		convertToRainbowMap(m_depth, m_out);
		break;
	default:
		m_depth.copyTo(m_out);
		break;
	}

	out_img.write(m_out);
}

/*
 * Based on OpenCv HighGUI
 */
void DepthConverter::convertToPointCloudMap(cv::Mat& data, cv::Mat& dataOut) {
	try{ 
		// !! This data are not exactly the same as when using opencv. Might need
		// some adjustments in the future. It's supposed to imitate the function
		// xnConvertProjectiveToRealWorld from OpenNI. Some attempt for this
		// was made in discussion on google groups
		// "ConvertProjectiveToRealWorld without camera connected Indstillinger"
		// To move further make use of content in the file cap_openni.cpp from OpenCv
		// and XnOpenNI.cpp from OpenNI
		// The whole idea behind is to transform coordinates and have them in real
		// world units (like meters)
	/*	double fx_d = 0.001 / 525;//5.9421434211923247e+02;
		double fy_d = 0.001 / 525;//5.9104053696870778e+02;
		double cx_d = 319.5;//3.1930780975300314e+02;
		double cy_d = 239.5;//2.4273913761751615e+02;*/

		// Check the presence of camera info.
		if (in_camera_info.empty())
			throw exception();

		// Retrieve camera info.
		Types::CameraInfo camera_info = in_camera_info.read();
		double fx_d = 0.001 / camera_info.fx();
		double fy_d = 0.001 / camera_info.fy();
		double cx_d = camera_info.cx();
		double cy_d = camera_info.cy();

		cv::Mat pointCloud_XYZ(camera_info.height(), camera_info.width(), CV_32FC3,
				cv::Scalar::all (INVALID_PIXEL));

		for (int y = 0; y < camera_info.height(); y++) {
			for (int x = 0; x < camera_info.width(); x++) {
				double depth = data.at<unsigned short>(y, x);
				// Check for invalid measurements
				if (depth == INVALID_PIXEL) // not valid
					pointCloud_XYZ.at < cv::Point3f > (y, x) = cv::Point3f(
							INVALID_COORDINATE, INVALID_COORDINATE,
							INVALID_COORDINATE);
				else {
					float xx = ((x - cx_d) * depth * fx_d);
					float yy = ((y - cy_d) * depth * fy_d);
					float dd = depth * 0.001f;
					float zz = sqrt(dd*dd-xx*xx-yy*yy);
					pointCloud_XYZ.at < cv::Point3f > (y, x) = cv::Point3f(xx, yy, dd);
				}
			}
		}

		pointCloud_XYZ.copyTo(dataOut);

	}catch(...){
		CLOG(LERROR) << "convertToPointCloudMap() requires camera info (in_camera_info)!\n";
	}
}

/*
 * Based on OpenCv HighGUI
 */
void DepthConverter::convertToDisparityMap(cv::Mat& data, cv::Mat& dataOut) {
	cv::Mat disp32;

	double mult = BASELINE * FOCAL_LENGTH;

	disp32.create(data.size(), CV_32FC1);
	disp32 = cv::Scalar::all(INVALID_PIXEL);
	for (int y = 0; y < disp32.rows; y++) {
		for (int x = 0; x < disp32.cols; x++) {
			unsigned short curDepth = data.at<unsigned short>(y, x);
			if (curDepth != INVALID_PIXEL)
				disp32.at<float>(y, x) = mult / curDepth;
		}
	}
	disp32.convertTo(dataOut, CV_8UC1);
}

void DepthConverter::convertToDisparityMap32f(cv::Mat& data, cv::Mat& dataOut) {
	double mult = BASELINE * FOCAL_LENGTH;

	dataOut.create(data.size(), CV_32FC1);
	dataOut = cv::Scalar::all(INVALID_PIXEL);
	for (int y = 0; y < dataOut.rows; y++) {
		for (int x = 0; x < dataOut.cols; x++) {
			unsigned short curDepth = data.at<unsigned short>(y, x);
			if (curDepth != INVALID_PIXEL)
				dataOut.at<float>(y, x) = mult / curDepth;
		}
	}
}

void DepthConverter::convertToValidPixelsMap(cv::Mat& data, cv::Mat& dataOut) {
	dataOut = (data != INVALID_PIXEL);
}

typedef struct{uchar r; uchar g; uchar b;} color;

void DepthConverter::convertToRainbowMap(cv::Mat& data, cv::Mat& dataOut) {
	cv::Mat out;

	out.create(data.size(), CV_8UC3);
	for (int y = 0; y < out.rows; y++) {
		for (int x = 0; x < out.cols; x++) {
			color col;
			col.r = col.g = col.b = 0;
			unsigned short curDepth = data.at<unsigned short>(y, x);
			int lb = curDepth & 0xff;
			if (curDepth > 50)
				switch (curDepth>>8) {
				case 0:
					col.b = 255;
					col.g = 255-lb;
					col.r = 255-lb;
					break;
				case 1:
					col.b = 255;
					col.g = lb;
					col.r = 0;
					break;
				case 2:
					col.b = 255-lb;
					col.g = 255;
					col.r = 0;
					break;
				case 3:
					col.b = 0;
					col.g = 255;
					col.r = lb;
					break;
				case 4:
					col.b = 0;
					col.g = 255-lb;
					col.r = 255;
					break;
				case 5:
					col.b = 0;
					col.g = 0;
					col.r = 255-lb;
					break;
				default:
					col.r = col.g = col.b = 0;
					break;
				}

			if (curDepth == INVALID_PIXEL)
				col.r=col.g=col.b = 0;
			out.at<color>(y, x) = col;
		}
	}
	out.copyTo(dataOut);
}

} //: namespace DepthConverter
} //: namespace Processors
