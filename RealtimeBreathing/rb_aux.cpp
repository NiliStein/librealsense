#include "rb_aux.h"
#include "os.h"
#include <cstdio>
#include <string>
#include <ctime>
#include <sstream>

void save_last_frame(const char* filename, const rs2::video_frame& frame) {
	static int frame_index = 0;
	static std::string frame_filenames[NUM_OF_LAST_FRAMES] = {""};

	std::string stream_desc{};
	std::string filename_base(filename);

	stream_desc = rs2_stream_to_string(frame.get_profile().stream_type());
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");

	auto filename_png = filename_base + "_" + stream_desc + oss.str() + ".png";

	// delete oldest frame file 
	if (frame_filenames[frame_index] != "") {
		const int result = remove(frame_filenames[frame_index].c_str());
		if (result != 0) {
			printf("%s\n", strerror(errno)); // No such file or directory
			// TODO: throw exception
		}
	}

	rs2::save_to_png(filename_png.data(), frame.get_width(), frame.get_height(), frame.get_bytes_per_pixel(),
		frame.get_data(), frame.get_width() * frame.get_bytes_per_pixel());

	frame_filenames[frame_index] = filename_png;
	frame_index = (frame_index + 1) % NUM_OF_LAST_FRAMES;
}