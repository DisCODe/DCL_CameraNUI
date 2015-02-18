/*!
 * \file
 * \brief
 */

#include <memory>
#include <string>

#include "CameraNUI.hpp"
#include "Common/Logger.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace Processors {
namespace CameraNUI {

CameraNUI::CameraNUI(const std::string & name) :
		Base::Component(name),
		sync("sync", true),
		m_camera_info(640, 480, 319.5, 239.5, 525, 525),
		index("index", 0),
		angle("angle", 0, "range"),
		triggered("triggered", false) {

	LOG(LTRACE)<< "Hello CameraNUI\n";
	registerProperty(sync);
	registerProperty(index);
	registerProperty(triggered);
	
	angle.addConstraint("-45");
	angle.addConstraint("45");
	registerProperty(angle);
}

CameraNUI::~CameraNUI() {
	LOG(LTRACE)<< "Good bye CameraNUI\n";
}

void CameraNUI::prepareInterface() {

	// Register data streams
	registerStream("trigger", &trigger);
	registerStream("out_img", &outImg);
	registerStream("out_depth", &outDepthMap);
	registerStream("out_camera_info", &camera_info);
	registerStream("in_angle", &in_angle);

  registerHandler("setAngle", boost::bind(&CameraNUI::setAngle, this));
  addDependency("setAngle", &in_angle);
  
  registerHandler("setAngleFromProperty", boost::bind(&CameraNUI::setAngleFromProperty, this));

	h_onNextImage.setup(this, &CameraNUI::onNextImage);
	registerHandler("onNextImage", &h_onNextImage);
	addDependency("onNextImage", NULL);
}

bool CameraNUI::onInit() {
	LOG(LTRACE)<< "CameraNUI::initialize\n";

	cameraFrame = cv::Mat(cv::Size(640,480),CV_8UC3,cv::Scalar(0));
	depthFrame = cv::Mat(cv::Size(640,480),CV_16UC1);

	return true;
}

bool CameraNUI::onFinish() {
	LOG(LTRACE)<< "CameraNUI::finish\n";
	return true;
}

void CameraNUI::onNextImage() {
	short *depth = 0;
	char *rgb = 0;
	uint32_t ts;
	int ret;

	if (triggered && trigger.empty()) {
		return;
	} else {
		// clear all triggers
		while(!trigger.empty()) trigger.read();
	}

	// retrieve color image
	ret = freenect_sync_get_video((void**)&rgb, &ts, index, FREENECT_VIDEO_RGB);
	cv::Mat tmp_rgb(480, 640, CV_8UC3, rgb);
	cv::cvtColor(tmp_rgb, cameraFrame, CV_RGB2BGR);

	// retrieve depth image
	ret = freenect_sync_get_depth((void**)&depth, &ts, index, FREENECT_DEPTH_REGISTERED);
	cv::Mat tmp_depth(480, 640, CV_16UC1, depth);
	tmp_depth.copyTo(depthFrame);

	// write data to output streams
	outImg.write(cameraFrame);
	outDepthMap.write(depthFrame);

	camera_info.write(m_camera_info);
}

void CameraNUI::setAngle() {
	int angle = in_angle.read();
	freenect_sync_set_tilt_degs(angle, index);
}

void CameraNUI::setAngleFromProperty() {
	CLOG(LNOTICE) << "Setting angle: " << angle;
	freenect_sync_set_tilt_degs(angle, index);
	freenect_raw_tilt_state * state;
	freenect_sync_get_tilt_state(&state, index);
	CLOG(LNOTICE) << "Angle: " << state->tilt_angle << ", state: " << state->tilt_status;
}

bool CameraNUI::onStop() {
	freenect_sync_stop();
	return true;
}

bool CameraNUI::onStart() {
	return true;
}

} //: namespace CameraNUI
} //: namespace Processors
