
#include "Webcam_RR.h"
#include <chrono>


namespace RR = RobotRaconteur;

namespace imaging = com::robotraconteur::imaging;
namespace image = com::robotraconteur::image;









Webcam::Webcam(VideoCapture capture,Webcam_RR_impl* reference) : imaging::Camera_default_impl()
{
	
	this->capture = capture;
	//this->imagedata = imagedata;
	this->reference = reference;
	image_width, image_height, image_step,image_encoding=0;
	this->streaming = false;
	//reference->cameras[cam_num] = this;
	
}

Webcam::~Webcam()
{
	this->stop_streaming();
	

}

image::ImagePtr Webcam::capture_frame() {
	boost::mutex::scoped_lock lock(this->reference->mtx_);
	//boost::lock_guard<boost::mutex> guard(this->multicam_reference->reference->mtx_);
	if (!this->streaming) {
		printf("Error camera must be streaming to capture frame");
		return 0;
	}
	Mat frame;
	this->capture.read(frame);
	image::ImagePtr imagedata(new image::Image());
	image::ImageInfoPtr image_info(new image::ImageInfo());
	
	image_info->height = frame.cols;
	image_info->width = frame.rows;
	image_info->step = frame.step;
	imagedata->image_info = image_info;

	imagedata->data = AttachRRArrayCopy(frame.data, *(frame.size.p));
		//std::cout << "sending image" << std::endl;
		
	return imagedata;
}

image::ImagePtr Webcam::capture_frame_threaded() {
	
	//boost::lock_guard<boost::mutex> guard(this->multicam_reference->reference->mtx_);
	if (!this->streaming) {
		printf("Error camera must be streaming to capture frame");
		return 0;
	}
	Mat frame;
	this->capture.read(frame);
	image::ImagePtr imagedata(new image::Image());
	image::ImageInfoPtr image_info(new image::ImageInfo());

	image_info->height = frame.cols;
	image_info->width = frame.rows;
	image_info->step = frame.step;
	imagedata->image_info = image_info;

	imagedata->data = AttachRRArrayCopy(frame.data, *(frame.size.p));
	//std::cout << "sending image" << std::endl;

	return imagedata;
}

void Webcam::start_streaming() {
	//initialize image data here so that it can be reused
	boost::lock_guard<boost::mutex> guard(this->reference->mtx_);
	//std::cout << (2^(this->camera_num)) << std::endl;
	if (!this->streaming) {
		
		this->streaming = true;
	}
}

void Webcam::stop_streaming() {
	
	this->streaming = false;
}

void Webcam::send_data(image::ImagePtr image_data) {
	image::ImageInfoPtr imageinfo(new image::ImageInfo());

	imageinfo->height = this->image_height;
	imageinfo->width = this->image_width;
	imageinfo->step = this->image_step;
	image_data->image_info = imageinfo;
	//image_data->image_info->encoding = this->image_encoding;
	this->rrvar_frame_stream->SendPacket(image_data);
}

void Webcam::set_frame_stream(PipePtr<image::ImagePtr> value) {
	imaging::Camera_default_impl::set_frame_stream(value);
	this->rrvar_frame_stream->SetMaxBacklog(3);
}


Webcam_RR_impl::Webcam_RR_impl(std::vector<webcaminputs> inputs)
{
	
	
	
	this->thread_exit = false;
	TIMEOUT_IN_MS = 100;
	//this->enabledSources = FrameSourceTypes_None;
	this->color_image_width = 3840;
	this->color_image_height = 2160;
	//std::cout << "Success" << std::endl;
	
	
	// Allocate memory for the different image streams (consider moving this to the enable_streams section?)
	
	
	this->color_image_data = new uint8_t[this->color_image_width * this->color_image_height];
	
	this->color_image_size = 0;
	//KinectPointCloud pointcloud (new KinectPointCloud(this));
	//this->pointcloud = KinectPointCloud(this);
	HRESULT hr = StartupWebcams(inputs);
	if (FAILED(hr))
	{
		std::cout << "Failed to Startup Kinect: error code " << HRESULT_CODE(hr) << std::endl;
		return;
	}
	
}

Webcam_RR_impl::~Webcam_RR_impl()
{
	ShutdownWebcams();
	
	delete color_image_data;
}

HRESULT Webcam_RR_impl::StartupWebcams(std::vector<webcaminputs> inputs)
{
	HRESULT hr = NULL;

	// Attempt access to default Kinect-2 sensor
	std::cout << "Looking for Default Kinect Sensor" << std::endl;
	//k4a_device  k4a_device_open(&this->kinect);
	
	std::vector<boost::shared_ptr<Webcam> > cameras;

	for (int t=0; t< inputs.size(); t++) {
		int apiID = cv::CAP_ANY;
		VideoCapture videocap;
		if (inputs[t].is_index) {
			

			videocap.open(inputs[t].index + apiID);
		}
		else {
			
			videocap = VideoCapture(inputs[t].camera_name, apiID);
			videocap.open(inputs[t].camera_name, apiID);
		}
		
		if (inputs[t].has_fps) {
			videocap.set(cv::CAP_PROP_FPS, inputs[t].fps);
		}
		if (inputs[t].has_resolution) {
			videocap.set(cv::CAP_PROP_FRAME_WIDTH, inputs[t].width);
			videocap.set(cv::CAP_PROP_FRAME_HEIGHT, inputs[t].height);
		}
		
		
		if (!videocap.isOpened()) {
			std::cerr << "ERROR! Unable to open camera index "<< inputs[t].index <<std::endl;

			return hr;
		}
		std::cout << "Opened camera index " << inputs[t].index << std::endl;
		cameras.push_back(boost::make_shared<Webcam>(videocap, this));
	}

	
	
		

		//hr = this->kinect->Open();
		//if FAILED(hr) { return hr; }
	
		
	std::cout << "Success" << std::endl;

		//this->enabledSources++;
		
		

		
	this->thread_exit = false;
	t1 = boost::thread(boost::bind(&Webcam_RR_impl::backgroundPollingThread, this));
	std::cout << "Success" << std::endl;
		
		
	

	return hr;
}

HRESULT Webcam_RR_impl::ShutdownWebcams()
{
	HRESULT hr = E_FAIL;

	this->enabledSources = 0;
	
	t1.interrupt();
	t1.join();

	for ( int i=0;i<this->cameras.size(); i++) {
		this->cameras[i]->capture.release();
		
	}
	
	return hr;
}




/*
void Webcam_RR_impl::CaptureArrived(k4a_capture_t* capture)
{
	
	
		//std::cout << "Looking at Color Frame Data" << std::endl;
		//IColorFrameReference *color_frame_reference = NULL;
		// IColorFrame *color_frame = NULL;
		//hr = multi_frame->get_ColorFrameReference(&color_frame_reference);
		// Acquire Frame
		
		//if SUCCEEDED(hr)
		//	hr = color_frame_reference->AcquireFrame(&color_frame);
	k4a_image_t color_frame = k4a_capture_get_color_image(*capture);

	if(color_frame)
	{
		if (this->enabledSources & 1) {
			image::ImagePtr imagedata(new image::Image());

			//std::cout << "Copying to buffer...";
			//this->mtx_.lock();
			
			//color_image_data = k4a_image_get_buffer(color_frame);
			this->color_image_size = k4a_image_get_size(color_frame);
			memcpy(this->color_image_data, (uint8_t*)(void*)k4a_image_get_buffer(color_frame), this->color_image_size);
			std::cout << this->color_image_size << std::endl;
			imagedata->data = AttachRRArrayCopy(this->color_image_data, this->color_image_size);
			//this->mtx_.unlock();
			this->multicamera->cameras[0]->send_data(imagedata);
		}
			
		//std::cout << ((SUCCEEDED(hr)) ? "Success" : "Failed") << std::endl;
	}

	

	
		
		

	
	
	
	//k4a_capture_release(*capture);
	
}
*/


void Webcam_RR_impl::backgroundPollingThread()
{
	//HRESULT hr;
	//DWORD res;
	std::cout << "Starting up background thread" << std::endl;
	while (!(this->thread_exit))
	{
		try
		{
			for (int i = 0; i < this->cameras.size(); i++) {
				boost::mutex::scoped_lock lock(this->mtx_);
				if (this->cameras[i]->streaming) {
					this->cameras[i]->send_data(this->cameras[i]->capture_frame_threaded());

				}
				//
			}
			
			//res = WaitForSingleObjectEx(reinterpret_cast<HANDLE>(h_event), 1000, false);
			
			//auto start = chrono::steady_clock::now();
			

		}
		catch (...) {
			break;
		}
	}

	std::cout << "Exiting Background Thread" << std::endl;
}