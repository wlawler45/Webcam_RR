#ifdef SendMessage
#undef SendMessage
#endif

#include <RobotRaconteur.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/opencv.hpp"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>


#include "robotraconteur_generated.h"

#include <Windows.h>
//#include <Kinect.h>
#include <iostream>
#include <boost/enable_shared_from_this.hpp>
#include <vector>
#include <string>
#include <cmath>
//#include <pcl/io/boost.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <opencv2/opencv.hpp>

#pragma once
using namespace cv;
using namespace RobotRaconteur;
using namespace boost;


namespace imaging = com::robotraconteur::imaging;
namespace image = com::robotraconteur::image;



	
	
class Webcam_RR_impl;

struct webcaminputs {
	bool is_index;
	int index;
	std::string camera_name;
	int fps;
	bool has_fps;
	bool has_resolution;
	int height;
	int width;
};
	
	
class Webcam : public imaging::Camera_default_impl, public boost::enable_shared_from_this<Webcam>
{

	image::ImagePtr imagedata;
public:
	
	Webcam(VideoCapture capture, Webcam_RR_impl* reference);
	Webcam_RR_impl* reference;
	~Webcam();
	int image_width, image_height, image_step;
	int image_encoding;
	void send_data(image::ImagePtr image_data);
	virtual  image::ImagePtr capture_frame();
	virtual  image::ImagePtr capture_frame_threaded();
	virtual void start_streaming();
	virtual void stop_streaming();
	virtual void set_frame_stream(PipePtr< image::ImagePtr> value);
	VideoCapture capture;
	bool streaming;

	
};
	







class Webcam_RR_impl
{
	
public:
	
	
	//multicamera would be parent device and tracker and pointcloud would be dependent on that
	std::vector<boost::shared_ptr<Webcam> > cameras;

	int color_image_size;
	int color_image_width, color_image_height;
	int num_cameras;
	uint8_t *color_image_data;
	
	boost::mutex mtx_;
	

	int enabledSources;
	Webcam_RR_impl(std::vector<webcaminputs> inputs);
	~Webcam_RR_impl();
	
	HRESULT StartupWebcams(std::vector<webcaminputs> inputs);
	HRESULT ShutdownWebcams();
	//virtual sensors::kinect2::ImagePtr getCurrentColorImage();
	//virtual sensors::kinect2::DepthImagePtr getCurrentDepthImage();
	//virtual sensors::kinect2::DepthImagePtr getCurrentInfraredImage();
	//virtual sensors::kinect2::ImagePtr getCurrentBodyIndexImage();
	//virtual sensors::kinect2::DepthImagePtr getCurrentLongExposureInfraredImage();
	

private:
	
	
	
	
	
	
	bool thread_exit;
	int TIMEOUT_IN_MS;
	//k4a_device_t kinect;
	
	//ICoordinateMapper *coordinate_mapper;

	//WAITABLE_HANDLE h_event;
	
	boost::thread t1;

	

	void backgroundPollingThread();

	template<class Interface> inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}
};