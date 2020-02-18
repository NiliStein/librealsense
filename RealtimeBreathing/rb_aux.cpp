#include "rb_aux.h"
#include "os.h"
#include <cstdio>
#include <string>
#include <ctime>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

//TODO: for logging
#include <fstream>

#define CALC_2D_DIST(name1,name2) { distance2D((*(name1))[0], (*(name1))[1], (*(name2))[0], (*(name2))[1]) }
#define CALC_3D_DIST(name1,name2) { distance3D((*(name1))[0], (*(name1))[1], (*(name1))[2], (*(name2))[0], (*(name2))[1], (*(name1))[2]) }
#define COORDINATES_TO_STRING(circle) (std::to_string(circle[0][0]) + ", " + std::to_string(circle[0][1]) + ", " + std::to_string(circle[0][2]))
#define NUM_OF_STICKERS 4

//TODO: for logging
std::ofstream logFile("frames\\log.txt");


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
	//TODO: stricter range, remove if the uncommented range below doesnt add noise
	//inRange(hsv8_mat, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), yellow_only_mask); //yellow bgr range
	inRange(hsv8_mat, cv::Scalar(20, 50, 50), cv::Scalar(40, 255, 255), yellow_only_mask); //yellow bgr range
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
	//GaussianBlur(yellow_only_grayscale_mat, yellow_only_grayscale_mat, cv::Size(9, 9), 2, 2);
	
	//cv::imwrite("frames\\yellow_grayscale_gaussian.jpg", yellow_only_grayscale_mat);

	//create binary image:
	cv::Mat image_th;
	cv::Mat bin_mat(yellow_only_grayscale_mat.size(), yellow_only_grayscale_mat.type());
	
	//TODO: simple threshold worked better than adaptive
	cv::threshold(yellow_only_grayscale_mat, image_th, 100, 255, cv::THRESH_BINARY);
	//TODO: remove?
	//cv::adaptiveThreshold(yellow_only_grayscale_mat, image_th, 255,
	//	cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 3, 5);
	
	cv::imwrite("frames\\yellow_grayscale_gaussian.jpg", image_th);

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
		return;
	}
	breathing_data->UpdateStickersLoactions();

	//calculate 2D distances:
	breathing_data->CalculateDistances2D();
	//add color timestamp:
	breathing_data->color_timestamp = color_frame.get_timestamp();

	//get depth of centroids:
	cv::Vec3f& left_ref = *(breathing_data->left);
	cv::Vec3f& right_ref = *(breathing_data->right);
	cv::Vec3f& middle_ref = *(breathing_data->middle);
	cv::Vec3f& down_ref = *(breathing_data->down);
	left_ref[2] = depth_frame.get_distance(left_ref[0],left_ref[1]);
	right_ref[2] = depth_frame.get_distance(right_ref[0], right_ref[1]);
	middle_ref[2] = depth_frame.get_distance(middle_ref[0], middle_ref[1]);
	down_ref[2] = depth_frame.get_distance(down_ref[0], down_ref[1]);

	//calculate 3D distances:
	breathing_data->CalculateDistances3D();
	//add depth timestamp:
	breathing_data->depth_timestamp = depth_frame.get_timestamp();

	//TODO: for logging
	logFile << breathing_data->GetDescription();
	
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
	if (circles.size() < NUM_OF_STICKERS) return;
	//sort vec by y:
	std::sort(circles.begin(), circles.end(), compareCirclesByYFunc);
	//sort 2 highest by x:
	std::sort(circles.begin(), circles.begin() + 2, compareCirclesByXFunc);

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
	dLR_depth = CALC_3D_DIST(left, right);
	dML_depth = CALC_3D_DIST(middle, left);
	dMR_depth = CALC_3D_DIST(middle, right);
	dMD_depth = CALC_3D_DIST(middle, down);
	dDL_depth = CALC_3D_DIST(down, left);
	dDR_depth = CALC_3D_DIST(down, right);

	//calculate average:
	average_3d_dist = (dLR_depth + dML_depth + dMR_depth + dMD_depth + dDL_depth + dDR_depth) / 6;

}

std::string BreathingFrameData::GetDescription() 
{
	std::string desc = "Color timestamp: " + std::to_string(color_timestamp) +
		"\nDepth timestamp: " + std::to_string(depth_timestamp) +
		"\nCoordinates left: " + COORDINATES_TO_STRING(left) +
		"\nCoordinates right: " + COORDINATES_TO_STRING(right) +
		"\nCoordinates middle: " + COORDINATES_TO_STRING(middle) +
		"\nCoordinates down: " + COORDINATES_TO_STRING(down) +
		"\n2D distances:" +
		"\nleft-right: " + std::to_string(dLR) +
		"\nmiddle-left: " + std::to_string(dML) +
		"\nmiddle-right: " + std::to_string(dMR) +
		"\nmiddle-down: " + std::to_string(dMD) +
		"\ndown-left: " + std::to_string(dDL) +
		"\ndown-right: " + std::to_string(dDR) +
		"\n3D distances:" +
		"\nleft-right: " + std::to_string(dLR_depth) +
		"\nmiddle-left: " + std::to_string(dML_depth) +
		"\nmiddle-right: " + std::to_string(dMR_depth) +
		"\nmiddle-down: " + std::to_string(dMD_depth) +
		"\ndown-left: " + std::to_string(dDL_depth) +
		"\ndown-right: " + std::to_string(dDR_depth) +
		"\n2D average distance: " + std::to_string(average_2d_dist) +
		"\n3D average distance: " + std::to_string(average_3d_dist) + "\n#################################################\n";
	return desc;
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
