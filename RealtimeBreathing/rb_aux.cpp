#include "rb_aux.h"
#include "os.h"
#include <librealsense2/rsutil.h>
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
#define COORDINATES_TO_STRING(circle) (std::to_string(circle[0][0]) + ", " + std::to_string(circle[0][1]) + ", " + std::to_string(circle[0][2]))
#define NUM_OF_STICKERS 4
#define CALC_2D_BY_CM true //if false, calculate by pixels
#define PI 3.14159265358979323846

//TODO: for logging
std::ofstream logFile("log.txt");

// fill out with n values, evely spaced (hopefully) between a and b, including a and b
void linespace(double a, double b, int n, std::vector<double>* out) {
	double step = (b - a) / (n - 1);
	for(int i = 0; i < n - 1; i++) {
		out->push_back(a);
		a += step;           // could recode to better handle rounding errors
	}
	out->push_back(b);
}

static float distance2D(float x, float y, float a, float b) {
	return sqrt(pow(x - a, 2) + pow(y - b, 2));
}

static float distance3D(float x, float y, float z, float a, float b, float c) {
	return sqrt(pow(x - a, 2) + pow(y - b, 2) + pow(z - c, 2));
}

void get_3d_coordinates(const rs2::depth_frame& depth_frame, float x, float y, cv::Vec3f& output) {
	
	float pixel[2] = { x, y };
	float point[3]; // From point (in 3D)
	auto dist = depth_frame.get_distance(pixel[0], pixel[1]);
	rs2_intrinsics intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
	rs2_deproject_pixel_to_point(point, &intr, pixel, dist);

	output[0] = float(point[0]) * 100.0;//convert to cm
	output[1] = float(point[1]) * 100.0;
	output[2] = float(point[2]) * 100.0;
}

void FFT(short int dir, long m, double *x, double *y)
{
	long n, i, i1, j, k, i2, l, l1, l2;
	double c1, c2, tx, ty, t1, t2, u1, u2, z;
	/* Calculate the number of points */
	n = 1;
	for (i = 0; i < m; i++)
		n *= 2;
	/* Do the bit reversal */
	i2 = n >> 1;
	j = 0;


	for (i = 0; i < n - 1; i++) {
		if (i < j) {
			tx = x[i];
			ty = y[i];
			x[i] = x[j];
			y[i] = y[j];
			x[j] = tx;
			y[j] = ty;
		}
		k = i2;
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}



	/* Compute the FFT */
	c1 = -1.0;
	c2 = 0.0;
	l2 = 1;
	for (l = 0; l < m; l++) {
		l1 = l2;
		l2 <<= 1;
		u1 = 1.0;
		u2 = 0.0;
		for (j = 0; j < l1; j++) {
			for (i = j; i < n; i += l2) {
				i1 = i + l1;
				t1 = u1 * x[i1] - u2 * y[i1];
				t2 = u1 * y[i1] + u2 * x[i1];
				x[i1] = x[i] - t1;
				y[i1] = y[i] - t2;
				x[i] += t1;
				y[i] += t2;
			}
			z = u1 * c1 - u2 * c2;
			u2 = u1 * c2 + u2 * c1;
			u1 = z;
		}
		c2 = sqrt((1.0 - c1) / 2.0);
		if (dir == 1)
			c2 = -c2;
		c1 = sqrt((1.0 + c1) / 2.0);
	}

	/* Scaling for forward transform */
    if (dir == 1) {
        for (i=0;i<n;i++) {
            x[i] /= n;
            y[i] /= n;
        }
    }
}

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

	//calculate 2D distances:
	breathing_data->CalculateDistances2D();

	//add color timestamp:
	breathing_data->color_timestamp = color_frame.get_timestamp();

	//calculate 3D distances:
	breathing_data->CalculateDistances3D();
	
	//add depth timestamp:
	breathing_data->depth_timestamp = depth_frame.get_timestamp();

	//add system timestamp:
	clock_t current_system_time = clock();
	breathing_data->system_timestamp = (current_system_time - manager_start_time) / double(CLOCKS_PER_SEC); //time in seconds elapsed since frame manager created

	//TODO: for logging
	logFile << breathing_data->GetDescription();

	//TODO: for debugging:
	if (this->interval_active) {
		std::string interval_text = "\n------------------- start interval ---------------------\n";
		logFile << interval_text;
	}
	
	add_frame_data(breathing_data);
	
	std::vector<cv::Point2d>* out = new std::vector<cv::Point2d>();
	//get_locations(stickers::left, out);
	
	get_dists(out);
	if (out->size() > 200) {
		long double f = calc_frequency_fft(out);
		logFile << "Frequency: " << f << " BPM: " << 60.0*f << "\n";
	}
	
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


void FrameManager::get_locations(stickers s, std::vector<cv::Point2d> *out) {
	if (user_cfg->mode == graph_mode::DISTANCES) {
		logFile << "Warning: get_locations was called in DISTANCES mode!\n";
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
		
	out->clear(); //clear the data

	double current_time = (clock() - manager_start_time) / double(CLOCKS_PER_SEC);
	for (unsigned int i = 0; i < _n_frames; i++) {
		int idx = (_oldest_frame_index + i + _n_frames) % _n_frames; //TODO: this is right order GIVEN that get_dists is run after add_frame_data (after _oldest_frame_idx++)
		if (_frame_data_arr[idx] != NULL) {
			//check if frame was received in under 15 sec
			double avg_dist = 0.0;
			int c = 0;
			double t = _frame_data_arr[idx]->system_timestamp; 
			for (std::pair<distances, bool> dist_elem : user_cfg->dists_included) {
				distances dist = dist_elem.first;
				bool is_included = dist_elem.second;
				if (is_included) { //if distance is included in user_cfg
					std::map<distances, float*>* distances_map = (user_cfg->dimension == dimension::D2) ? &_frame_data_arr[idx]->distances_map_2d : &_frame_data_arr[idx]->distances_map_3d;
					avg_dist += *((*distances_map)[dist]);
					c += 1;
				}
			}
				
			avg_dist = avg_dist / (1.0*c);
			out->push_back(cv::Point2d(t, avg_dist));
		}
	}
}

// assuming dists and time are ordered according to time (oldest fisrt)
long double FrameManager::cal_frequency_dft(std::vector<cv::Point2d>* samples) {
	
	int N = samples->size();	// N - number of samples (frames)
	
	double t0 = samples->at(0).x;	// t0, t1 - start, end time(seconds)
	double t1 = samples->at(N-1).x;
	
	int fps = N / (t1 - t0);	// FPS - frames per second
	// t - vector of time signatures
		// note: t = [t0    t0+(t1-t0/win_size-1)    t0+(t1-t0/win_size-1)*2    t0+(t1-t0/win_size-1)*3 ...... t1];
	//std::vector<double> t;
	//linespace(t0, t1, N, &t);

	/*
	// vector of frequency signatures
	// note: frequency is  a vector of(N / 2) evenly spaced points between 0 and FPS / 2.
	std::vector<double> frequencies;
	linespace(0, fps / 2, N / 2, &frequencies);
	*/
	//std::vector<double> coef;
	
	double C = 2 * PI / N;
	int max_idx = 0;
	double max_val = 0;
	int mink = (4.0 / 60.0)*((N - 2.0) / fps);	// assume BPM >= 4
	//calculate real part of coefficients
	for (int k = mink; k <= N / 2; k++) {	//frequency 0 always most dominant. ignore first coef.
		double coef_re = 0.0;
		double coef_im = 0.0;
		for (int n = 0; n < N; n++) {
			coef_re += samples->at(n).y * cos(C*k*n);
			coef_im += samples->at(n).y * sin(C*k*n);
		}
		double abs_val = coef_re * coef_re + coef_im * coef_im;
		if (abs_val > max_val) {
			max_val = abs_val;
			max_idx = k;
		}
	}
	
	float f = fps / (N - 2.0) * max_idx;
	logFile << "method get_frequency (dft):\n";
	logFile << "\nfrequency = fps / (N - 2.0) * max_idx: " << fps << " / (" << N << " - 2.0) * " << max_idx <<  " = " << f << '\n';

	return f;
}

long double FrameManager::calc_frequency_fft(std::vector<cv::Point2d>* samples) {
	
	int realSamplesNum = samples->size();	// N - number of samples (frames)
	//const int paddedSamplesNum = 512;	// fft works requires that the number of samples is a power of 2 &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	const int paddedSamplesNum = 256;	// fft works requires that the number of samples is a power of 2
	int dir = 1;
	long m = log2(paddedSamplesNum);
	double X[paddedSamplesNum] = { 0 };
	for (int s = 0; s < realSamplesNum; s++) { // insert real samples in first #realSamplesNum slots
		X[s] = samples->at(s).y;
	}
	double Y[paddedSamplesNum] = { 0 };	// no imaginary part in samples
	double t0 = samples->at(0).x;	// t0, t1 - start, end time(seconds)
	double t1 = samples->at(realSamplesNum - 1).x;
	int fps = realSamplesNum / (t1 - t0);	// FPS - frames per second
	
	FFT(dir, m, X, Y);
	
	/*const int top = 100;
	int top_max_idx[top] = { 0 };
	double min_bpm = 7.0;
	int mini = ceil((min_bpm / 60.0)*((paddedSamplesNum - 2.0) / fps));	// assume BPM >= min_bpm
	for (int j = 0; j < top; j++) {
		int max_idx = 0;
		double max_val = 0;
		for (int i = mini; i < realSamplesNum / 2; i++) { //frequency 0 always most dominant. ignore first coef.
			double val = abs(X[i])*abs(X[i]) + abs(Y[i])*abs(Y[i]);
			if (val > max_val) {
				max_val = val;
				max_idx = i;
			}
		}
		top_max_idx[j] = max_idx;
		X[max_idx] = 0;
	}
	double avg_max_idx = 0;
	for (int i = 0; i < top; i++)  avg_max_idx += top_max_idx[i];
	avg_max_idx /= top;
	logFile << "method get_frequency_fft:\n";
	logFile << "fps: " << fps << '\n';
	logFile << "realSamplesNum: " << realSamplesNum << '\n';
	long double f = fps / (paddedSamplesNum - 2.0) * top_max_idx[0];
	long double f_avg = fps / (paddedSamplesNum - 2.0) * avg_max_idx;
	logFile << "frequency = fps / (paddedSamplesNum - 2.0) * max_idx: " << fps << " / (" << paddedSamplesNum << " - 2.0) * " << top_max_idx[0] << " = " << f << '\n';
	logFile << "frequency avg of two = fps / (paddedSamplesNum - 2.0) * avg_max_idx: " << fps << " / (" << paddedSamplesNum << " - 2.0) * " << avg_max_idx << "  = " << f_avg << '\n';
	logFile << "    Frequency: " << f << "     BPM: " << 60.0*f << "\n";
	logFile << "avg Frequency: " << f_avg << " avg BPM: " << 60.0*f_avg << "\n";
	*/
	
	int max_idx = 0;
	double max_val = 0;
	double min_bpm = 6.0;	// TODO: decide minimal BPM

	if (fps == 0) {
		return 0;
	}
	int mini = ceil((min_bpm / 60.0)*((paddedSamplesNum - 2.0) / fps));	// assume BPM >= min_bpm
	for (int i = mini; i < realSamplesNum / 2; i++) { //frequency 0 always most dominant. ignore first coef.
		double val = abs(X[i])*abs(X[i]) + abs(Y[i])*abs(Y[i]); 
		if (val > max_val) {
			max_val = val;
			max_idx = i;
		}
	}
	logFile << "method get_frequency_fft:\n";
	logFile << "fps: " << fps << '\n';
	logFile << "realSamplesNum: " << realSamplesNum << '\n';
	long double f = fps / (paddedSamplesNum - 2.0) * max_idx;
	logFile << "frequency = fps / (paddedSamplesNum - 2.0) * max_idx: " << fps << " / (" << paddedSamplesNum << " - 2.0) * " << max_idx << " = " << f << '\n';

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

void BreathingFrameData::CalculateDistances2D()
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
	if (NUM_OF_STICKERS == 5) {
		average_2d_dist = (dLM1 + dLM2 + dLM3 + dLR + dRM1 + dRM2 + dRM3 + dM1M2 + dM1M3 + dM2M3) / 10;
	}
	else {
		average_2d_dist = (dLM2 + dLM3 + dLR + dRM2 + dRM3 + dM2M3) / 6;
	}

}



void BreathingFrameData::CalculateDistances3D()
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
	if (NUM_OF_STICKERS == 5) {
		average_3d_dist = (dLM1_depth + dLM2_depth + dLM3_depth + dLR_depth + dRM1_depth + dRM2_depth + dRM3_depth + dM1M2_depth + dM1M3_depth + dM2M3_depth) / 10;
	}
	else {
		average_3d_dist = (dLM2_depth + dLM3_depth + dLR_depth + dRM2_depth + dRM3_depth + dM2M3_depth) / 6;
	}

}

std::string BreathingFrameData::GetDescription() 
{
	std::string desc = "Color timestamp: " + std::to_string(color_timestamp) +
		"\nDepth timestamp: " + std::to_string(depth_timestamp) +
		"\nSystem timestamp: " + std::to_string(system_timestamp) +
		"\nCoordinates left: " + COORDINATES_TO_STRING((&left_cm)) +
		"\nCoordinates right: " + COORDINATES_TO_STRING((&right_cm));
	if (NUM_OF_STICKERS == 5) desc += "\nCoordinates mid1: " + COORDINATES_TO_STRING((&mid1_cm));
	desc += "\nCoordinates mid2: " + COORDINATES_TO_STRING((&mid2_cm)) +
		"\nCoordinates mid3: " + COORDINATES_TO_STRING((&mid3_cm)) +
		"\n2D distances:";

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
		"\n2D average distance: " + std::to_string(average_2d_dist) +
		"\n3D average distance: " + std::to_string(average_3d_dist) + "\n#################################################\n";
	return desc;
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
	
	if (line.compare("D") == 0) {
		Config::mode = graph_mode::DISTANCES;
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
	}
}

GraphPlot::GraphPlot(FrameManager& frame_manager) {
	CvPlot::Axes* axes = new CvPlot::Axes(CvPlot::makePlotAxes());
	axes->setXLimAuto();
	axes->setYLim(std::pair<double, double>(0, 40));
	//axes->create<CvPlot::Series>(data, ".b").setName("data series");
	window = new CvPlot::Window("Breathing Graph", *axes);
	clock_t current_system_time = clock();
	time_begin = (current_system_time - frame_manager.manager_start_time) / double(CLOCKS_PER_SEC);
}

GraphPlot::~GraphPlot() {

}

void GraphPlot::updateGraphPlot(FrameManager& frame_manager) {
	std::vector<cv::Point2d> points;
	frame_manager.get_dists(&points);
	CvPlot::Axes& axes = window->getAxes();
	axes.find<CvPlot::Series>("data series")->setPoints(points);
	window->update();
}

//void GraphPlot::plot(FrameManager& frame_manager) {
//	if (first) {
//		//window = new CvPlot::Window("a", axes, 600, 800);
//		auto &axes = window->getAxes();
//		axes.setXLim(std::pair<double, double>(time_begin, time_begin + 90));
//		axes.setYLim(std::pair<double, double>(0, 40));
//		std::vector<cv::Point2d> points;
//		frame_manager.get_dists(&points);
//		first = false;
//	}
//	else {
//		updateGraphPlot(frame_manager);
//	}
//
//}


void GraphPlot::updatePlotLoc(FrameManager& frame_manager) {
	std::vector<cv::Point2d> points;
	frame_manager.get_locations(stickers::left, &points); //TODO: get sticker from user_cfg
	auto &axes = window->getAxes();
	axes.find<CvPlot::Series>("location series")->setPoints(points);
	window->update();
}

void GraphPlot::plotLoc(FrameManager& frame_manager) {
	if (first) {
		//window = new CvPlot::Window("Depth", axes, 600, 800);
		auto &axes = window->getAxes();
		//create the series for the first time:
		axes.create<CvPlot::Series>(".g").setName("location series");
		axes.setXLim(std::pair<double, double>(time_begin, time_begin + 90));
		axes.setYLim(std::pair<double, double>(0, 40));
		first = false;
	}
	else {
		updatePlotLoc(frame_manager);
	}

}

long double GraphPlot::plotDists(FrameManager& frame_manager) {
	if (first) {
		//window = new CvPlot::Window("Distances", axes, 600, 800);
		auto &axes = window->getAxes();
		//create the series for the first time:
		axes.create<CvPlot::Series>(".b").setName("distances series");
		axes.setXLim(std::pair<double, double>(time_begin, time_begin + 90));
		axes.setYLim(std::pair<double, double>(0, 40));
		first = false;
		return 0;
	}
	else {
		return updatePlotDists(frame_manager);
	}

}

long double GraphPlot::updatePlotDists(FrameManager& frame_manager) {
	std::vector<cv::Point2d> points;
	frame_manager.get_dists(&points);
	long double f = (frame_manager.calc_frequency_fft(&points));
	CvPlot::Axes& axes = window->getAxes();
	axes.find<CvPlot::Series>("distances series")->setPoints(points);
	window->update();
	return f;
}

void showGraph::operator()(GraphPlot* graph, FrameManager* frame_manager) {
	while (true) {
		if (frame_manager->get_frames_array_size() > 20)
			graph->plotDists(*frame_manager);
	}
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
