#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#define NUM_OF_LAST_FRAMES 150 //max number of 15 seconds under 30 fps
#define CONFIG_FILEPATH "config.txt"

// OLD:
//void save_last_frame(const char* filename, const rs2::video_frame& frame);

enum graph_mode {
	DISTANCES,
	LOCATION
};

enum stickers {
	left,
	mid1,
	mid2,
	mid3,
	right,
	sdummy // needed for enum iteration
};

enum distances {
	left_mid1,
	left_mid2,
	left_mid3,
	left_right,
	right_mid1,
	right_mid2,
	right_mid3,
	mid1_mid2,
	mid1_mid3,
	mid2_mid3,
	ddummy // needed for enum iteration
};

/* 
	Configuration details extracted from config.txt
	@ mode: indeicates the kind of graph to be presented.
		distances - tracking a given set of distances. bpm will be calculated using the average of said set.
		location - tracking the location of a given set of stickers. (TODO: no bpm calculated for this mode?)
	@ stickers_included: set of stickers to include. 
		for mode distances, TODO: any sticker required for an included distance? or it wont be used, tbd 
		for mode location, all stickers of which the location is requested.
	@ dists_included: set of distances to be tracked
		only used in mode distances.
*/
class Config {
public:
	graph_mode mode;
	std::map<stickers, bool> stickers_included;
	std::map<distances, bool> dists_included;

	//ctor:
	Config(const char* config_filepath);
};

/*	BreathingFrameData class
	Stores the processed data of a frame.
	@ circles : saves the centroids of the circles found representing the stickers.
		Each circle has x,y coordinates and depth, indexes of each are in the corresponding order.
		First circle is the frame center.
	@ left, right, mid1-3 : Pointers to the coordinates of the corresponding sticker, as following:
		left ---- mid1 ---- right
		  -					 -
		    -	  mid2	   -
			  -			 -
			    - mid3 -
		Values can be invalid if stickers do not exist. Careful!
	@ ***_cm : coordinates in cm.
	@ dAB : distance in pixels between A and B, while A,B are initials of the names of the stickers.
	@ dAB_depth : 3D distance in cm between A and B, while A,B are initials of the names of the stickers.
	@ average_2d_dist, average_3d_dist : average distances between stickers, in 2D and in 3D.
	@ ***_timestamp : timestamp of each frame.
*/
class BreathingFrameData {
public:
	std::vector<cv::Vec3f> circles; //(x,y,depth)
	cv::Vec3f *left, *right, *mid1, *mid2, *mid3; //(x,y,depth)
	//TODO: adding alternative coordinates in cm, choose one after deciding which is more accurate (dist by pixels or by cm(given by rs2_deproject))
	cv::Vec3f left_cm, right_cm, mid1_cm, mid2_cm, mid3_cm; //(x,y,depth)
	float dLM1, dLM2, dLM3, dLR, dRM1, dRM2, dRM3, dM1M2, dM1M3, dM2M3;
	//float dLR, dML, dMR, dMD, dDL, dDR;
	float dLM1_depth, dLM2_depth, dLM3_depth, dLR_depth, dRM1_depth, dRM2_depth, dRM3_depth, dM1M2_depth, dM1M3_depth, dM2M3_depth;
	//float dLR_depth, dML_depth, dMR_depth, dMD_depth, dDL_depth, dDR_depth;
	float average_2d_dist;
	float average_3d_dist;
	double color_timestamp;
	double depth_timestamp;

	//ctor:
	BreathingFrameData() :
		left(NULL), right(NULL), mid1(NULL), mid2(NULL), mid3(NULL),
		dLM1(0.0), dLM2(0.0), dLM3(0.0), dLR(0.0), dRM1(0.0), dRM2(0.0), dRM3(0.0), dM1M2(0.0), dM1M3(0.0), dM2M3(0.0),
		//dLR(0.0), dML(0.0), dMR(0.0), dMD(0.0), dDL(0.0), dDR(0.0),
		dLM1_depth(0.0), dLM2_depth(0.0), dLM3_depth(0.0), dLR_depth(0.0), dRM1_depth(0.0), dRM2_depth(0.0), dRM3_depth(0.0), dM1M2_depth(0.0), dM1M3_depth(0.0), dM2M3_depth(0.0),
		//dLR_depth(0.0), dML_depth(0.0), dMR_depth(0.0), dMD_depth(0.0), dDL_depth(0.0), dDR_depth(0.0),
		average_2d_dist(0.0), average_3d_dist(0.0),
		color_timestamp(0.0), depth_timestamp(0.0)
	{}

	//METHODS://

	/* Updates the locations of the stickers and validates the pointers to them. */
	void UpdateStickersLoactions();

	/* Calculates 2D distances between all stickers and their average. */
	void CalculateDistances2D();
	/* Calculates 3D distances between all stickers and their average. */
	void CalculateDistances3D();

	/* Gets the description of the frame in the following format:
		TODO: update format */
	std::string GetDescription();
};


/* FrameManager class.
	Manages all frames and the memory required for them.
*/
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
	void process_frame(const rs2::video_frame& color_frame, const rs2::depth_frame& depth_frame);

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

	Config user_cfg;
	unsigned int _n_frames;
	unsigned int _oldest_frame_index;
	BreathingFrameData** _frame_data_arr;
	const char* _frame_disk_path;
};