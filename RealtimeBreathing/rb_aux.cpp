#include "rb_aux.h"
#include "os.h"
#include <cstdio>
#include <string>
#include <ctime>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#define CALC_2D_DIST(name1,name2) { distance2D((*(name1))[0], (*(name1))[1], (*(name2))[0], (*(name2))[1]) }
#define CALC_3D_DIST(name1,depth1,name2,depth2) { distance3D((*(name1))[0], (*(name1))[1], (depth1), (*(name2))[0], (*(name2))[1], (depth2)) }

static float distance2D(float x, float y, float a, float b) {
	return sqrt(pow(x - a, 2) + pow(y - b, 2));
}

static float distance3D(float x, float y, float z, float a, float b, float c) {
	return sqrt(pow(x - a, 2) + pow(y - b, 2) + pow(z - c, 2));
}

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

//void process_frame_data(cv::Mat& frame_bgr8_mat, double color_timestamp) {
//	static FrameDataVec frame_data_vec;
//	std::vector<BreathingFrameData>* breathing_frame_data_vec = &(frame_data_vec.data_vec);
//	BreathingFrameData new_frame_data;
//
//	//find yellows:
//	Mat yellow_only_mat;
//	inRange(frame_bgr8_mat, Scalar(0, 180, 255), Scalar(170, 255, 255), yellow_only_mat); //yellow bgr range
//	Mat yellow_only_grayscale_mat;
//
//	//Convert it to gray
//	cvtColor(yellow_only_mat, yellow_only_grayscale_mat, COLOR_BGR2GRAY);
//
//	//find circles:
//	//Reduce the noise so we avoid false circle detection
//	GaussianBlur(yellow_only_grayscale_mat, yellow_only_grayscale_mat, Size(9, 9), 2, 2);
//	//Apply Hough:
//	HoughCircles(yellow_only_grayscale_mat, new_frame_data.circles, HOUGH_GRADIENT,
//		2,   // accumulator resolution (size of the image / 2)
//		5,  // minimum distance between two circles
//		100, // Canny high threshold
//		100, // minimum number of votes
//		0, 1000); // min and max radius
//
//
//	//distinguish between stickers:
//	if (new_frame_data.circles.size < 4) //no circles found
//		// TODO: Cleanup
//		return;
//	new_frame_data.UpdateStickersLoactions();
//
//	//calculate distances:
//	new_frame_data.CalculateDistances2D();
//	new_frame_data.color_timestamp = color_timestamp;
//	//add timestamp:
//	breathing_frame_data_vec->push_back(new_frame_data);
//}

FrameManager::FrameManager(unsigned int n_frames, const char * frame_disk_path) :
	_n_frames(n_frames), _frame_disk_path(frame_disk_path), _oldest_frame_index(0)
{
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

void FrameManager::process_color_frame(const rs2::video_frame& color_frame)
{

	// Create bgr mode matrix of color_frame
	const void * color_frame_data = color_frame.get_data();
	cv::Mat rgb8_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame_data, cv::Mat::AUTO_STEP);
	cv::Mat hsv8_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3);
	cv::cvtColor(rgb8_mat, hsv8_mat, cv::COLOR_RGB2HSV);

	BreathingFrameData * breathing_data = new BreathingFrameData();

	//find yellows:
	cv::Mat yellow_only_mask(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC1);
	inRange(hsv8_mat, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), yellow_only_mask); //yellow bgr range
	cv::Mat yellow_only_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, cv::Scalar(0, 0, 0));
	hsv8_mat.copyTo(yellow_only_mat, yellow_only_mask);

	// TODO: For Debug only, remove when finished
	cv::imwrite("frames\\only_yellow.jpg", yellow_only_mat);


	//Pick up the grayscale
	cv::Mat yellow_only_bgr8_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3);
	cvtColor(yellow_only_mat, yellow_only_bgr8_mat, cv::COLOR_HSV2BGR);
	cv::Mat yellow_only_grayscale_mat(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3);
	cvtColor(yellow_only_bgr8_mat, yellow_only_grayscale_mat, cv::COLOR_BGR2GRAY);

	cv::imwrite("frames\\yellow_grayscale.jpg", yellow_only_grayscale_mat);

	//find circles:
	//Reduce the noise so we avoid false circle detection
	GaussianBlur(yellow_only_grayscale_mat, yellow_only_grayscale_mat, cv::Size(9, 9), 2, 2);
	//Apply Hough:
	HoughCircles(yellow_only_grayscale_mat, breathing_data->circles, cv::HOUGH_GRADIENT,
		2,   // accumulator resolution (size of the image / 2)
		5,  // minimum distance between two circles
		100, // Canny high threshold
		100, // minimum number of votes
		0, 1000); // min and max radius

	//distinguish between stickers:
	if (breathing_data->circles.size() < 4) //no circles found
		cleanup();
		return;
	breathing_data->UpdateStickersLoactions();

	//calculate distances:
	breathing_data->CalculateDistances2D();
	//add timestamp:
	breathing_data->color_timestamp = color_frame.get_timestamp();

	add_frame_data(breathing_data);
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

void BreathingFrameData::UpdateStickersLoactions()
{
	if (circles.size() < 4) return;
	//sort vec by y:
	std::sort(circles.begin(), circles.end(), compareCirclesByYFunc);
	//sort 2 highest by x:
	std::sort(circles.begin(), circles.begin() + 1, compareCirclesByXFunc);

	left = &circles[0];
	right = &circles[1];
	middle = &circles[2];
	down = &circles[3];
}

void BreathingFrameData::CalculateDistances2D()
{
	if (!left || !right || !middle || !down) return;
	dLR = CALC_2D_DIST(left, right);
	dML = CALC_2D_DIST(middle, left);
	dMR = CALC_2D_DIST(middle, right);
	dMD = CALC_2D_DIST(middle, down);
	dDL = CALC_2D_DIST(down, left);
	dDR = CALC_2D_DIST(down, right);

	//calculate average:
	average_2d_dist = (dLR + dML + dMR + dMD + dDL + dDR) / 6;
}

void BreathingFrameData::CalculateDistances3D()
{
	if (!left || !right || !middle || !down) return;
	dLR_depth = CALC_3D_DIST(left, left_depth, right, right_depth);
	dML_depth = CALC_3D_DIST(middle, middle_depth, left, left_depth);
	dMR_depth = CALC_3D_DIST(middle, middle_depth, right, right_depth);
	dMD_depth = CALC_3D_DIST(middle, middle_depth, down, down_depth);
	dDL_depth = CALC_3D_DIST(down, down_depth, left, left_depth);
	dDR_depth = CALC_3D_DIST(down, down_depth, right, right_depth);

	//calculate average:
	average_3d_dist = (dLR_depth + dML_depth + dMR_depth + dMD_depth + dDL_depth + dDR_depth) / 6;

}

void save_last_frame(const char* filename, const rs2::video_frame& frame) {
	static int frame_index = 0;
	static int frame_counter = 0;
	static std::string frame_filenames[NUM_OF_LAST_FRAMES] = { "" };

	std::string stream_desc{};
	std::string filename_base(filename);

	stream_desc = rs2_stream_to_string(frame.get_profile().stream_type());
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");

	auto filename_png = filename_base + "_" + stream_desc + oss.str() + std::to_string(frame_counter) + ".png";

	// delete oldest frame file 
	if (frame_filenames[frame_index] != "") {
		const int result = remove(frame_filenames[frame_index].c_str());
		if (result != 0) {
			printf("remove(%s) failed. Error: %s\n", frame_filenames[frame_index].c_str(), strerror(errno)); // No such file or directory
			// TODO: throw exception
		}
	}

	rs2::save_to_png(filename_png.data(), frame.get_width(), frame.get_height(), frame.get_bytes_per_pixel(),
		frame.get_data(), frame.get_width() * frame.get_bytes_per_pixel());

	frame_filenames[frame_index] = filename_png;
	frame_index = (frame_index + 1) % NUM_OF_LAST_FRAMES;
	frame_counter++;
}
