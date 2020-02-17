#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#define NUM_OF_LAST_FRAMES 10

void save_last_frame(const char* filename, const rs2::video_frame& frame);

class BreathingFrameData {
public:
	std::vector<cv::Vec3f> circles;
	cv::Vec3f *left, *right, *middle, *down;
	float left_depth, right_depth, middle_depth, down_depth;
	float dLR, dML, dMR, dMD, dDL, dDR;
	float dLR_depth, dML_depth, dMR_depth, dMD_depth, dDL_depth, dDR_depth;
	float average_2d_dist;
	float average_3d_dist;
	double color_timestamp;
	double depth_timestamp;

	//ctor:
	BreathingFrameData() :
		left(NULL), right(NULL), middle(NULL), down(NULL),
		dLR(0.0), dML(0.0), dMR(0.0), dMD(0.0), dDL(0.0), dDR(0.0),
		dLR_depth(0.0), dML_depth(0.0), dMR_depth(0.0), dMD_depth(0.0), dDL_depth(0.0), dDR_depth(0.0),
		average_2d_dist(0.0), average_3d_dist(0.0),
		color_timestamp(0.0), depth_timestamp(0.0)
	{}

	//METHODS://

	void UpdateStickersLoactions();
	void CalculateDistances2D();
	void CalculateDistances3D();
	std::string GetDescription();
};

class FrameManager {
public:
	//ctor
	FrameManager(unsigned int n_frames = NUM_OF_LAST_FRAMES, const char * frame_disk_path = NULL);

	//dtor
	~FrameManager();

	/**
	 * Processes a video color frame
	 *
	 * @param frame - a video frame from the camera
	 *
	 */
	void process_color_frame(const rs2::video_frame& color_frame);

private:
	/**
	 * Cleans all allocated resources
	 */
	void cleanup();

	/**
	 * Add frame_data to collection of frame datas.
	 * NOTE: only last n_frames saved so the oldest frame_data will be deleted 
	 */
	void add_frame_data(BreathingFrameData * frame_data);

	unsigned int _n_frames;
	unsigned int _oldest_frame_index;
	BreathingFrameData** _frame_data_arr;
	const char* _frame_disk_path;
};