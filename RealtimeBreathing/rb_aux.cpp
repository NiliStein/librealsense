#include "rb_aux.h"
#include "utilities.h"
#include "os.h"
#include <cstdio>
#include <string>
#include <ctime>
#include <sstream>
//#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
//#include <CvPlot/cvplot.h> // in .h file

//TODO: for logging
#include <fstream>

#define CALC_2D_DIST(name1,name2) { distance2D((*(name1))[0], (*(name1))[1], (*(name2))[0], (*(name2))[1]) }
#define CALC_3D_DIST(name1,name2) { distance3D(name1[0], name1[1], name1[2], name2[0], name2[1], name2[2]) }
#define COORDINATES_TO_STRING_CM(circle) (std::to_string(circle[0]) + ", " + std::to_string(circle[1]) + ", " + std::to_string(circle[2]))
#define COORDINATES_TO_STRING_PIXELS(circle) (std::to_string(circle[0][0]) + ", " + std::to_string(circle[0][1]))


#define NUM_OF_STICKERS 5
#define CALC_2D_BY_CM false	//if false, calculate by pixels
#define GET_FREQUENCY_BY_FFT true	//if false, use get_frequency_differently



/* Compare functions to sort the stickers: */

static struct compareCirclesByY {
	//returns true if c2's y is greater than c1's y.
	bool operator() (cv::Vec3f& c1, cv::Vec3f& c2) {
		return (c1[1] < c2[1]);
	}
} compareCirclesByYFunc;

static struct compareCirclesByX {
	//returns true if c2's x is greater than c1's x.
	bool operator() (cv::Vec3f& c1, cv::Vec3f& c2) {
		return (c1[0] < c2[0]);
	}
} compareCirclesByXFunc;

FrameManager::FrameManager(Config* cfg, unsigned int n_frames, const char * frame_disk_path) :
	_n_frames(n_frames), _frame_disk_path(frame_disk_path), _oldest_frame_index(0), user_cfg(cfg), interval_active(false)
{
	manager_start_time = clock();
	_frame_data_arr = new BreathingFrameData*[_n_frames];
	for (unsigned int i = 0; i < _n_frames; i++) {
		_frame_data_arr[i] = NULL;
	}
}

FrameManager::~FrameManager()
{
	cleanup();
	if (_frame_data_arr != NULL) {
		free(_frame_data_arr);
	}
}

void FrameManager::restart() {
	frame_idx = 1;
	first_timestamp = NULL;
	_oldest_frame_index = 0;
	interval_active = false;
	manager_start_time = clock();
	for (unsigned int i = 0; i < _n_frames; i++) {
		_frame_data_arr[i] = NULL;
	}
}

int FrameManager::get_frames_array_size() {
	int c = 0;
	for (unsigned int i = 0; i < _n_frames; i++) {
		if (_frame_data_arr[i] != NULL) {
			c++;
		}
	}
	return c;
}

void FrameManager::process_frame(const rs2::video_frame& color_frame, const rs2::depth_frame& depth_frame)
{
	// Create bgr mode matrix of color_frame
	const void * color_frame_data = color_frame.get_data();
	cv::Mat rgb8_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame_data, cv::Mat::AUTO_STEP);
	cv::Mat hsv8_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3);
	cv::cvtColor(rgb8_mat, hsv8_mat, cv::COLOR_RGB2HSV);

	BreathingFrameData * breathing_data = new BreathingFrameData();

	//find yellows:
	cv::Mat yellow_only_mask(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC1);
	//hsv official yellow range: (20, 100, 100) to (30, 255, 255)
	inRange(hsv8_mat, cv::Scalar(20, 50, 50), cv::Scalar(40, 255, 255), yellow_only_mask); //yellow hsv range
	cv::Mat yellow_only_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, cv::Scalar(0, 0, 0));
	hsv8_mat.copyTo(yellow_only_mat, yellow_only_mask);

	// TODO: For Debug only, remove when finished
	//cv::imwrite("frames\\only_yellow.jpg", yellow_only_mat);


	//Pick up the grayscale
	cv::Mat yellow_only_bgr8_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3);
	cvtColor(yellow_only_mat, yellow_only_bgr8_mat, cv::COLOR_HSV2BGR);
	cv::Mat yellow_only_grayscale_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3);
	cvtColor(yellow_only_bgr8_mat, yellow_only_grayscale_mat, cv::COLOR_BGR2GRAY);
	
	// TODO: For Debug only, remove when finished
	//cv::imwrite("frames\\yellow_grayscale.jpg", yellow_only_grayscale_mat);
	
	//create binary image:
	cv::Mat image_th;
	cv::Mat bin_mat(yellow_only_grayscale_mat.size(), yellow_only_grayscale_mat.type());
	
	// simple threshold worked better than adaptive
	cv::threshold(yellow_only_grayscale_mat, image_th, 100, 255, cv::THRESH_BINARY);
	//TODO: remove?
	//cv::adaptiveThreshold(yellow_only_grayscale_mat, image_th, 255,
	//	cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 3, 5);
	
	//cv::imwrite("frames\\yellow_grayscale_gaussian.jpg", image_th);

	//connected components:
	cv::Mat1i labels;
	cv::Mat1i stats;
	cv::Mat1d centroids;
	cv::connectedComponentsWithStats(image_th, labels, stats, centroids);
	
	//calc threshold for connected component area (max area/2)
	int area_threshold = 0;
	for (int i = 1; i < stats.rows; i++) //label 0 is the background
	{
		if (stats[i][cv::CC_STAT_AREA] > area_threshold) {
			area_threshold = stats[i][cv::CC_STAT_AREA];
		}

	}
	area_threshold = area_threshold / 2;

	//get centers:
	for (int i = 1; i < centroids.rows; i++) //label 0 is the background
	{
		if (stats[i][cv::CC_STAT_AREA] > area_threshold) {
			breathing_data->circles.push_back(cv::Vec3f(centroids(i, 0), centroids(i, 1)));
			circle(rgb8_mat, cv::Point(centroids(i, 0), centroids(i, 1)), 3, cv::Scalar(0, 255, 0));
		}
	}

	//distinguish between stickers:
	if (breathing_data->circles.size() < NUM_OF_STICKERS) {//not all circles were found
		cleanup();
		interval_active = false; //missed frame so calculation has to start all over.
		return;
	}
	breathing_data->UpdateStickersLoactions();

	//get 3D cm coordinates
	get_3d_coordinates(depth_frame, float((*breathing_data->left)[0]), (*breathing_data->left)[1], breathing_data->left_cm);
	get_3d_coordinates(depth_frame, float((*breathing_data->right)[0]), (*breathing_data->right)[1], breathing_data->right_cm);
	if (NUM_OF_STICKERS == 5) {
		get_3d_coordinates(depth_frame, float((*breathing_data->mid1)[0]), (*breathing_data->mid1)[1], breathing_data->mid1_cm);
	}
	get_3d_coordinates(depth_frame, float((*breathing_data->mid2)[0]), (*breathing_data->mid2)[1], breathing_data->mid2_cm);
	get_3d_coordinates(depth_frame, float((*breathing_data->mid3)[0]), (*breathing_data->mid3)[1], breathing_data->mid3_cm);
	
	
	
	
	//if user config dimension is 3D, check for 0,-0,0 3d coordinates. dump such frames
	if (user_cfg->dimension == dimension::D3) {
		if (check_illegal_3D_coordinates(breathing_data)) {
			frames_dumped_in_row++;
			logFile << "Warning: illegal 3D coordinates! frame was dumped.\n";
			if (frames_dumped_in_row >= 3) cleanup();
			return;
		}
		else {
			frames_dumped_in_row = 0;
		}
	}
	//calculate 2D distances:
	breathing_data->CalculateDistances2D(user_cfg);

	//calculate 3D distances:
	breathing_data->CalculateDistances3D(user_cfg);

	//add color timestamp:
	breathing_data->color_timestamp = color_frame.get_timestamp();	
	
	//add depth timestamp:
	breathing_data->depth_timestamp = depth_frame.get_timestamp();

	//add system timestamp:
	clock_t current_system_time = clock();
	breathing_data->system_timestamp = (current_system_time - manager_start_time) / double(CLOCKS_PER_SEC); //time in seconds elapsed since frame manager created

	
	if (!first_timestamp) first_timestamp = (breathing_data->color_timestamp < breathing_data->depth_timestamp) ? breathing_data->color_timestamp : breathing_data->depth_timestamp;
	breathing_data->frame_idx = frame_idx; 
	frame_idx++;
	breathing_data->color_idx = color_frame.get_frame_number();
	breathing_data->depth_idx = depth_frame.get_frame_number();
	breathing_data->system_color_timestamp = (breathing_data->color_timestamp - first_timestamp) / double(CLOCKS_PER_SEC); //time elapsed from first timestamp in video - which timestamp?
	breathing_data->system_depth_timestamp = (breathing_data->depth_timestamp - first_timestamp) / double(CLOCKS_PER_SEC);

	//TODO: for logging
	//logFile << breathing_data->GetDescription();	//&&&&&&&&&&&&&&&
	breathing_data->GetDescription_temp();

	/*
	//TODO: for debugging:
	if (this->interval_active) {
		std::string interval_text = "\n------------------- start interval ---------------------\n";
		logFile << interval_text;
	}
	*/
	//check if frame id duplicated
	bool is_dup = false;
	int last_frame_index = (_n_frames + _oldest_frame_index - 1) % _n_frames;
	if (_frame_data_arr[last_frame_index] != NULL) {
		if (user_cfg->dimension == dimension::D2) {
			if (_frame_data_arr[last_frame_index]->color_timestamp == breathing_data->color_timestamp
				&& _frame_data_arr[last_frame_index]->average_2d_dist == breathing_data->average_2d_dist) {
				is_dup = true;
			}
		}
		if (user_cfg->dimension == dimension::D3) {
			if (_frame_data_arr[last_frame_index]->color_timestamp == breathing_data->color_timestamp
				&& _frame_data_arr[last_frame_index]->depth_timestamp == breathing_data->depth_timestamp
				&& _frame_data_arr[last_frame_index]->average_3d_dist == breathing_data->average_3d_dist) {
				is_dup = true;
			}
		}
		

	}
	if (!is_dup) add_frame_data(breathing_data);
	
}

void FrameManager::cleanup()
{
	if (_frame_data_arr != NULL) {

		for (unsigned int i = 0; i < _n_frames; i++) {
			if (_frame_data_arr[i] != NULL) {
				free(_frame_data_arr[i]);
			}
			_frame_data_arr[i] = NULL;
		}
	}
	logFile << "frames array cleanup...\n";
	frames_dumped_in_row = 0;
}

void FrameManager::add_frame_data(BreathingFrameData * frame_data)
{
	// delete last frame
	if (_frame_data_arr[_oldest_frame_index] != NULL) {
		free(_frame_data_arr[_oldest_frame_index]);
		_frame_data_arr[_oldest_frame_index] = NULL;
	}

	_frame_data_arr[_oldest_frame_index] = frame_data;
	_oldest_frame_index = (_oldest_frame_index + 1) % _n_frames;
}


void FrameManager::get_locations(stickers s, std::vector<cv::Point2d> *out) {
	if (user_cfg->mode != graph_mode::LOCATION) {
		logFile << "Warning: get_locations was called in incompatible mode! (use L mode)\n";
		return;
	}
	if (_frame_data_arr == NULL) return;	
	
	double current_time = (clock() - manager_start_time) / double(CLOCKS_PER_SEC);
	for (unsigned int i = 0; i < _n_frames; i++) {
		int idx = (_oldest_frame_index + i + _n_frames) % _n_frames; //TODO: this is right order GIVEN that get_locations is run after add_frame_data (after _oldest_frame_idx++)
		if (_frame_data_arr[idx] != NULL) {
			//check if frame was received in under 15 sec
			if ((current_time - _frame_data_arr[idx]->system_timestamp) <= 15.0) { //TODO: decide if needed
				out->push_back(cv::Point2d(_frame_data_arr[idx]->system_timestamp, (*_frame_data_arr[idx]->stickers_map_cm[s])[2]));
			}
		}
	}
}

void FrameManager::get_dists(std::vector<cv::Point2d>* out) {
	if (user_cfg->mode == graph_mode::LOCATION) {
		logFile << "Warning: get_dists was called in LOCATION mode!\n";
		return;
	}
	if (_frame_data_arr == NULL) return;
		
	double current_time = (clock() - manager_start_time) / double(CLOCKS_PER_SEC);
	for (unsigned int i = 0; i < _n_frames; i++) {
		int idx = (_oldest_frame_index + i + _n_frames) % _n_frames; //TODO: this is right order GIVEN that get_dists is run after add_frame_data (after _oldest_frame_idx++)
		if (_frame_data_arr[idx] != NULL) {
			double avg_dist = (user_cfg->dimension == dimension::D2) ? _frame_data_arr[idx]->average_2d_dist : _frame_data_arr[idx]->average_3d_dist;
			double t = _frame_data_arr[idx]->system_timestamp; 
			out->push_back(cv::Point2d(t, avg_dist));
		}
	}
}

long double FrameManager::no_graph() {
	std::vector<cv::Point2d> points;
	get_dists(&points);
	long double f;
	if (GET_FREQUENCY_BY_FFT) {
		normalize_distances(&points);
		f = (calc_frequency_fft(&points));
	}
	else {	// get_frequency_differently
		bool cm_units = (user_cfg->dimension == dimension::D3) ? true : CALC_2D_BY_CM;
		f = calc_frequency_differently(&points, cm_units);
	}
	return f;
}

void FrameManager::activateInterval() {
	this->interval_active = true;
}

void FrameManager::deactivateInterval() {
	this->interval_active = false;
}

void BreathingFrameData::UpdateStickersLoactions()
{
	if (circles.size() < NUM_OF_STICKERS) return;
	if (circles.size() == 4) {
		// no mid1 sticker
		//sort vec by y:
		std::sort(circles.begin(), circles.end(), compareCirclesByYFunc);
		//sort 2 highest by x:
		std::sort(circles.begin(), circles.begin() + 2, compareCirclesByXFunc);

		left = &circles[0];
		right = &circles[1];
		mid2 = &circles[2];
		mid3 = &circles[3];
	}
	else {
		// assume 5 stickers, arranged in a T shape
		//sort vec by y:
		std::sort(circles.begin(), circles.end(), compareCirclesByYFunc);
		//sort 3 highest by x:
		std::sort(circles.begin(), circles.begin() + 3, compareCirclesByXFunc);

		left = &circles[0];
		right = &circles[2];
		mid1 = &circles[1];
		mid2 = &circles[3];
		mid3 = &circles[4];
	}
	
}

void BreathingFrameData::CalculateDistances2D(Config* user_cfg)
{
	if (!left || !right || !mid2 || !mid3) return;
	if (NUM_OF_STICKERS == 5 && !mid1) return;

	if (CALC_2D_BY_CM) {
		dLM2 = CALC_2D_DIST((&left_cm), (&mid2_cm));
		dLM3 = CALC_2D_DIST((&left_cm), (&mid3_cm));
		dLR = CALC_2D_DIST((&left_cm), (&right_cm));
		dRM2 = CALC_2D_DIST((&right_cm), (&mid2_cm));
		dRM3 = CALC_2D_DIST((&right_cm), (&mid3_cm)); 
		dM2M3 = CALC_2D_DIST((&mid2_cm), (&mid3_cm));
		if (NUM_OF_STICKERS == 5) {		//sticker mid1 exists
			dLM1 = CALC_2D_DIST((&left_cm), (&mid1_cm));
			dRM1 = CALC_2D_DIST((&right_cm), (&mid1_cm));
			dM1M2 = CALC_2D_DIST((&mid1_cm), (&mid2_cm));
			dM1M3 = CALC_2D_DIST((&mid1_cm), (&mid3_cm));
		}
	}
	else {	// calc by pixels
		dLM2 = CALC_2D_DIST(left, mid2);
		dLM3 = CALC_2D_DIST(left, mid3);
		dLR = CALC_2D_DIST(left, right);
		dRM2 = CALC_2D_DIST(right, mid2);
		dRM3 = CALC_2D_DIST(right, mid3);
		dM2M3 = CALC_2D_DIST(mid2, mid3);
		if (NUM_OF_STICKERS == 5) {		//sticker mid1 exists
			dLM1 = CALC_2D_DIST(left, mid1);
			dRM1 = CALC_2D_DIST(right, mid1);
			dM1M2 = CALC_2D_DIST(mid1, mid2);
			dM1M3 = CALC_2D_DIST(mid1, mid3);
		}
	}
	
	//calculate average:
	average_2d_dist = 0.0;
	int c = 0;
	for (std::pair<distances, bool> dist_elem : user_cfg->dists_included) {
		distances dist = dist_elem.first;
		bool is_included = dist_elem.second;
		if (is_included) { //if distance is included in user_cfg
			average_2d_dist += *(distances_map_2d[dist]);
			c += 1;
		}
	}

		average_2d_dist = average_2d_dist / (1.0*c);
}



void BreathingFrameData::CalculateDistances3D(Config* user_cfg)
{
	if (!left || !right || !mid2 || !mid3) return;
	if (NUM_OF_STICKERS == 5 && !mid1) return;

	dLM2_depth = CALC_3D_DIST(left_cm, mid2_cm); 
	dLM3_depth = CALC_3D_DIST(left_cm, mid3_cm); 
	dLR_depth = CALC_3D_DIST(left_cm, right_cm); 
	dRM2_depth = CALC_3D_DIST(right_cm, mid2_cm);
	dRM3_depth = CALC_3D_DIST(right_cm, mid3_cm);
	dM2M3_depth = CALC_3D_DIST(mid2_cm, mid3_cm);

	if (NUM_OF_STICKERS == 5) {		//sticker mid1 exists
		dLM1_depth = CALC_3D_DIST(left_cm, mid1_cm);
		dRM1_depth = CALC_3D_DIST(right_cm, mid1_cm);
		dM1M2_depth = CALC_3D_DIST(mid1_cm, mid2_cm);
		dM1M3_depth = CALC_3D_DIST(mid1_cm, mid3_cm);
	}

	//calculate average:
	average_3d_dist = 0.0;
	int c = 0;
	for (std::pair<distances, bool> dist_elem : user_cfg->dists_included) {
		distances dist = dist_elem.first;
		bool is_included = dist_elem.second;
		if (is_included) { //if distance is included in user_cfg
			average_3d_dist += *(distances_map_3d[dist]);
			c += 1;
		}
	}

	average_3d_dist = average_3d_dist / (1.0*c);
}

std::string BreathingFrameData::GetDescription() 
{
	const std::string d2method = (CALC_2D_BY_CM) ? "cm" : "pixels";
	std::string desc = "#################################################\nFrame idx: " + std::to_string(frame_idx);
	desc += "\nColor idx : " + std::to_string(color_idx);
	desc += "\nDepth idx : " + std::to_string(depth_idx);
	desc += "\nColor timestamp: " + std::to_string(color_timestamp) +
		"\nDepth timestamp: " + std::to_string(depth_timestamp) +
		"\nSystem timestamp: " + std::to_string(system_timestamp) +
		"\nSystem color timestamp: " + std::to_string(system_color_timestamp) +
		"\nSystem depth timestamp: " + std::to_string(system_depth_timestamp) +
		"\nCoordinates cm:\n" + 
		"left: " + COORDINATES_TO_STRING_CM(left_cm) +
		"\nright: " + COORDINATES_TO_STRING_CM(right_cm);
	if (NUM_OF_STICKERS == 5) desc += "\nmid1: " + COORDINATES_TO_STRING_CM(mid1_cm);
	desc += "\nmid2: " + COORDINATES_TO_STRING_CM(mid2_cm) +
		"\nmid3: " + COORDINATES_TO_STRING_CM(mid3_cm) +
		"\nCoordinates pixels:\n" + 
		"left: " + COORDINATES_TO_STRING_PIXELS(left) +
		"\nright: " + COORDINATES_TO_STRING_PIXELS(right);
	if (NUM_OF_STICKERS == 5) desc += "\nmid1: " + COORDINATES_TO_STRING_PIXELS(mid1);
	desc += "\nmid2: " + COORDINATES_TO_STRING_PIXELS(mid2) +
		"\nmid3: " + COORDINATES_TO_STRING_PIXELS(mid3) +
	"\n2D distances (" + d2method + "):";

	if (NUM_OF_STICKERS == 5)  desc += "\nleft-mid1: " + std::to_string(dLM1);
	desc += "\nleft-mid2: " + std::to_string(dLM2) +
		"\nleft-mid3: " + std::to_string(dLM3) +
		"\nleft-right: " + std::to_string(dLR);
	if (NUM_OF_STICKERS == 5)  desc += "\nright-mid1: " + std::to_string(dRM1);
	desc += "\nright-mid2: " + std::to_string(dRM2) +
		"\nright-mid3: " + std::to_string(dRM3);
	if (NUM_OF_STICKERS == 5) {
		desc += "\nmid1-mid2: " + std::to_string(dM1M2) +
			"\nmid1-mid3: " + std::to_string(dM1M3);
	}
	desc += "\nmid2-mid3: " + std::to_string(dM2M3) +
		"\n3D distances:";

	if (NUM_OF_STICKERS == 5)  desc += "\nleft-mid1: " + std::to_string(dLM1_depth);
	desc += "\nleft-mid2: " + std::to_string(dLM2_depth) +
		"\nleft-mid3: " + std::to_string(dLM3_depth) +
		"\nleft-right: " + std::to_string(dLR_depth);
	if (NUM_OF_STICKERS == 5)  desc += "\nright-mid1: " + std::to_string(dRM1_depth);
	desc += "\nright-mid2: " + std::to_string(dRM2_depth) +
		"\nright-mid3: " + std::to_string(dRM3_depth);
	if (NUM_OF_STICKERS == 5) {
		desc += "\nmid1-mid2: " + std::to_string(dM1M2_depth) +
			"\nmid1-mid3: " + std::to_string(dM1M3_depth);
	}
	desc += "\nmid2-mid3: " + std::to_string(dM2M3_depth) +
		"\n2D average distance: " + std::to_string(average_2d_dist) + " " + d2method +	
		"\n3D average distance: " + std::to_string(average_3d_dist) + " cm\n";		
	return desc;
}

void BreathingFrameData::GetDescription_temp()
{
	const std::string d2method = (CALC_2D_BY_CM) ? "cm" : "pixels";

	logFile << "#################################################\nFrame idx: " << std::to_string(frame_idx);
	logFile << "\nColor idx : " << std::to_string(color_idx);
	logFile << "\nDepth idx : " + std::to_string(depth_idx);
	logFile << "\nColor timestamp: " + std::to_string(color_timestamp) <<
		"\nDepth timestamp: " << std::to_string(depth_timestamp) <<
		"\nSystem color timestamp: " << std::to_string(system_color_timestamp) <<
		"\nSystem depth timestamp: " << std::to_string(system_depth_timestamp) <<
		"\nCoordinates cm:\n" << "left: " << std::fixed << std::setprecision(2) << left_cm[0] << " " << std::fixed << std::setprecision(2) << left_cm[1] << " " << std::fixed << std::setprecision(2) << left_cm[2];
	logFile <<  "\nright: " << std::fixed << std::setprecision(2) << right_cm[0] << " " << std::fixed << std::setprecision(2) << right_cm[1] << " " << std::fixed << std::setprecision(2) << right_cm[2];
	if (NUM_OF_STICKERS == 5) logFile << "\nmid1: " << std::fixed << std::setprecision(2) << mid1_cm[0] << " " << std::fixed << std::setprecision(2) << mid1_cm[1] << " " << std::fixed << std::setprecision(2) << mid1_cm[2];
	logFile << "\nmid2: " << std::fixed << std::setprecision(2) << mid2_cm[0] << " " << std::fixed << std::setprecision(2) << mid2_cm[1] << " " << std::fixed << std::setprecision(2) << mid2_cm[2];
	logFile <<	"\nmid3: " << std::fixed << std::setprecision(2) << mid3_cm[0] << " " << std::fixed << std::setprecision(2) << mid3_cm[1] << " " << std::fixed << std::setprecision(2) << mid3_cm[2];
	logFile <<	"\nCoordinates pixels:\n" <<
		"left: " <<std::fixed << std::setprecision(2) << (*left)[0] << " " << std::fixed << std::setprecision(2) << (*left)[1] <<
		"\nright: " << std::setprecision(2) << (*right)[0] << " " << std::setprecision(2) << (*right)[1];
	if (NUM_OF_STICKERS == 5) logFile << "\nmid1: " << std::fixed << std::setprecision(2) << (*mid1)[0] << " " << std::fixed << std::setprecision(2) << (*mid1)[1];
	logFile << "\nmid2: " << std::fixed << std::setprecision(2) << (*mid2)[0] << " " << std::fixed << std::setprecision(2) << (*mid2)[1];
	logFile << "\nmid3: " << std::fixed << std::setprecision(2) << (*mid3)[0] << " " << std::fixed << std::setprecision(2) << (*mid3)[1];
	logFile <<"\n2D distances (" << d2method << "):";

	if (NUM_OF_STICKERS == 5)  logFile << "\nleft-mid1: " << std::fixed << std::setprecision(2) << dLM1;
	logFile << "\nleft-mid2: " << std::fixed << std::setprecision(2) << dLM2 <<
		"\nleft-mid3: " << std::fixed << std::setprecision(2) << dLM3 <<
		"\nleft-right: " << std::fixed << std::setprecision(2) << dLR;
	if (NUM_OF_STICKERS == 5)  logFile << "\nright-mid1: " << std::fixed << std::setprecision(2) << dRM1;
	logFile << "\nright-mid2: " << std::fixed << std::setprecision(2) << dRM2 <<
		"\nright-mid3: " << std::fixed << std::setprecision(2) << dRM3;
	if (NUM_OF_STICKERS == 5) {
		logFile << "\nmid1-mid2: " << std::fixed << std::setprecision(2) << dM1M2 <<
			"\nmid1-mid3: " << std::fixed << std::setprecision(2) << dM1M3;
	}
	logFile << "\nmid2-mid3: " << std::fixed << std::setprecision(2) << dM2M3 <<
		"\n3D distances:";

	if (NUM_OF_STICKERS == 5)  logFile << "\nleft-mid1: " << std::fixed << std::setprecision(2) << dLM1_depth;
	logFile << "\nleft-mid2: " << std::fixed << std::setprecision(2) << dLM2_depth <<
		"\nleft-mid3: " << std::fixed << std::setprecision(2) << dLM3_depth <<
		"\nleft-right: " << std::fixed << std::setprecision(2) << dLR_depth;
	if (NUM_OF_STICKERS == 5)  logFile << "\nright-mid1: " << std::fixed << std::setprecision(2) << dRM1_depth;
	logFile << "\nright-mid2: " << std::fixed << std::setprecision(2) << dRM2_depth <<
		"\nright-mid3: " << std::setprecision(2) << dRM3_depth;
	if (NUM_OF_STICKERS == 5) {
		logFile << "\nmid1-mid2: " << std::fixed << std::setprecision(2) << dM1M2_depth <<
			"\nmid1-mid3: " << std::fixed << std::setprecision(2) << dM1M3_depth;
	}
	logFile << "\nmid2-mid3: " << std::fixed << std::setprecision(2) << dM2M3_depth <<
		"\n2D average distance: " << std::fixed << std::setprecision(6) << average_2d_dist <<
		"\n3D average distance: " << std::fixed << std::setprecision(6) << average_3d_dist << "\n";	 
}


Config::Config(const char* config_filepath) {
	std::ifstream config_file(config_filepath);
	std::string line;
	//get dimension
	std::getline(config_file, line); //first line is a comment
	std::getline(config_file, line);
	Config::dimension =  (line.compare("2") == 0) ? dimension::D2 : dimension::D3;
	//get mode
	std::getline(config_file, line); //next line is a comment
	std::getline(config_file, line);
	
	if (line.compare("D") == 0 || line.compare("F") == 0 || line.compare("N") == 0) {
		Config::mode = (line.compare("D") == 0) ? graph_mode::DISTANCES : (line.compare("F") == 0) ? graph_mode::FOURIER : graph_mode::NOGRAPH;
		std::getline(config_file, line); // new line
		std::getline(config_file, line); // comment
		// get distances to include
		for (int distInt = distances::left_mid1; distInt != distances::ddummy; distInt++) {
			distances d = static_cast<distances>(distInt);
			std::getline(config_file, line);
			//std::string k = line.substr(0, line.find(" "));
			std::string val = line.substr(line.length() - 1, line.length());
			Config::dists_included[d] = (val.compare("y") == 0) ? true : false;
		}
		//if NUM_OF_STICKERS is 4, there is no mid1 sticker
		if (NUM_OF_STICKERS == 4) {
			if (dists_included[distances::left_mid1] || dists_included[distances::mid1_mid2] ||
				dists_included[distances::mid1_mid3] || dists_included[distances::right_mid1]) {
				logFile << "Warning: distance from mid1 was set to y, while number of stickers is 4. This distance will be disregarded.\n";
				dists_included[distances::left_mid1] = dists_included[distances::mid1_mid2] =
					dists_included[distances::mid1_mid3] = dists_included[distances::right_mid1] = false;
			}
		}
	}
	else {
		Config::mode = graph_mode::LOCATION;
		std::getline(config_file, line); // new line
		std::getline(config_file, line); // comment
		//skip distances
		getline(config_file, line);
		while (line.substr(0, 1).compare("#") != 0) getline(config_file, line);
		// get included stickers
		for (int stInt = stickers::left; stInt != stickers::sdummy; stInt++) {
			stickers s = static_cast<stickers>(stInt);
			std::getline(config_file, line);
			std::string val = line.substr(line.length() - 1, line.length());
			Config::stickers_included[s] = (val.compare("y") == 0) ? true : false;
		}
		//if NUM_OF_STICKERS is 4, there is no mid1 sticker
		if (NUM_OF_STICKERS == 4) {
			if (stickers_included[stickers::mid1]) {
				logFile << "Warning: location of mid1 was set to y, while number of stickers is 4. This location will be disregarded.\n";
				stickers_included[stickers::mid1] = false;
			}
		}
	}
}

GraphPlot::GraphPlot(FrameManager& frame_manager) {
	axes = CvPlot::makePlotAxes();
	clock_t current_system_time = clock();
	time_begin = (current_system_time - frame_manager.manager_start_time) / double(CLOCKS_PER_SEC);
}


void GraphPlot::restart(FrameManager& frame_manager) {
	first = true;
	clock_t current_system_time = clock();
	time_begin = (current_system_time - frame_manager.manager_start_time) / double(CLOCKS_PER_SEC);
	Dx_LOWER_BOUND = time_begin;
	Dx_UPPER_BOUND = time_begin + 90;
	Lx_LOWER_BOUND = time_begin;
	Lx_UPPER_BOUND = time_begin + 90;
	axes = CvPlot::makePlotAxes();
	delete window;
}

long double GraphPlot::plotFourier(FrameManager& frame_manager) {
	if (first) {
		window = new CvPlot::Window("Fourier", axes, 600, 800);
		axes.setXLim(std::pair<double, double>(Fx_LOWER_BOUND, Fx_UPPER_BOUND));
		axes.setYLim(std::pair<double, double>(Fy_LOWER_BOUND, Fy_UPPER_BOUND));
		//get_samples_from_file(&alons_samples); //&&&&&&&&&&&&&&&&&alons doll log
		first = false;
	}
	return updatePlotFourier(frame_manager);
}

long double GraphPlot::updatePlotFourier(FrameManager& frame_manager) {
	std::vector<cv::Point2d> dist_points;
	frame_manager.get_dists(&dist_points);	
	normalize_distances(&dist_points);
	std::vector<cv::Point2d> frequency_points;
	//simulate_running_window(running_index, &alons_samples, &dist_points); //&&&&&&&&&&&&&&&&&alons doll log
	//running_index++;	//&&&&&&&&&&&&&&&&&alons doll log
	long double f = (calc_frequency_fft(&dist_points, &frequency_points));
	axes = CvPlot::makePlotAxes();
	axes.setXLim(std::pair<double, double>(Fx_LOWER_BOUND, Fx_UPPER_BOUND));
	axes.setYLim(std::pair<double, double>(Fy_LOWER_BOUND, Fy_UPPER_BOUND));
	if (frequency_points.size() > 50) {
		//frequency_points[0] = cv::Point2d(0, 0);	//&&&&&&&& 
		axes.create<CvPlot::Series>(frequency_points, "-k");
		window->update();
	}
	return f;
}

void GraphPlot::plotLoc(FrameManager& frame_manager) {
	if (first) {
		window = new CvPlot::Window("Depth", axes, 600, 800);
		axes.setXLim(std::pair<double, double>(Lx_LOWER_BOUND, Lx_UPPER_BOUND));
		axes.setYLim(std::pair<double, double>(Ly_LOWER_BOUND, Ly_UPPER_BOUND));
		first = false;
	}
	updatePlotLoc(frame_manager);
}

void GraphPlot::updatePlotLoc(FrameManager& frame_manager) {
	std::vector<cv::Point2d> points[5];
	const std::string lineSpec[5] = { "-k", "-g", "-b", "-r", "-y" };
	//axes = CvPlot::makePlotAxes();
	for (int stInt = stickers::left; stInt != stickers::sdummy; stInt++) {
		stickers s = static_cast<stickers>(stInt);
		if (frame_manager.user_cfg->stickers_included[s]) {
			frame_manager.get_locations(s, &points[stInt]);
			axes.create<CvPlot::Series>(points[stInt], lineSpec[stInt]);
		}
	}
	window->update();
}

long double GraphPlot::plotDists(FrameManager& frame_manager) {
	if (first) {
		window = new CvPlot::Window("Distances", axes, 600, 800);
		axes.setXLim(std::pair<double, double>(Dx_LOWER_BOUND, Dx_UPPER_BOUND));
		axes.setYLim(std::pair<double, double>(Dy_LOWER_BOUND, Dy_UPPER_BOUND));
		first = false;
	}
	return updatePlotDists(frame_manager);
}

long double GraphPlot::updatePlotDists(FrameManager& frame_manager) {
	std::vector<cv::Point2d> points;
	frame_manager.get_dists(&points);
	//axes = CvPlot::makePlotAxes(); //needed?
	axes.create<CvPlot::Series>(points, "-b");
	window->update();
	long double f;
	if (GET_FREQUENCY_BY_FFT) {
		normalize_distances(&points);
		f = (calc_frequency_fft(&points));
	}
	else {	// get_frequency_differently
		bool cm_units = (frame_manager.user_cfg->dimension==dimension::D3) ? true : CALC_2D_BY_CM;
		f = calc_frequency_differently(&points, cm_units);															
	}
	return f;
}

/* OLD FUNCTIONS: */

//void save_last_frame(const char* filename, const rs2::video_frame& frame) {
//	static int frame_index = 0;
//	static int frame_counter = 0;
//	static std::string frame_filenames[NUM_OF_LAST_FRAMES] = { "" };
//
//	std::string stream_desc{};
//	std::string filename_base(filename);
//
//	stream_desc = rs2_stream_to_string(frame.get_profile().stream_type());
//	auto t = std::time(nullptr);
//	auto tm = *std::localtime(&t);
//	std::ostringstream oss;
//	oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
//
//	auto filename_png = filename_base + "_" + stream_desc + oss.str() + std::to_string(frame_counter) + ".png";
//
//	// delete oldest frame file 
//	if (frame_filenames[frame_index] != "") {
//		const int result = remove(frame_filenames[frame_index].c_str());
//		if (result != 0) {
//			printf("remove(%s) failed. Error: %s\n", frame_filenames[frame_index].c_str(), strerror(errno)); // No such file or directory
//			// TODO: throw exception
//		}
//	}
//
//	rs2::save_to_png(filename_png.data(), frame.get_width(), frame.get_height(), frame.get_bytes_per_pixel(),
//		frame.get_data(), frame.get_width() * frame.get_bytes_per_pixel());
//
//	frame_filenames[frame_index] = filename_png;
//	frame_index = (frame_index + 1) % NUM_OF_LAST_FRAMES;
//	frame_counter++;
//}
