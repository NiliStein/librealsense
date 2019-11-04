#include "rb_aux.h"
#include "os.h"

void save_last_frame(const char* filename, const rs2::video_frame& frame) {
	std::string stream_desc{};
	std::string filename_base(filename);

	stream_desc = rs2_stream_to_string(frame.get_profile().stream_type());
	auto filename_png = filename_base + "_" + stream_desc + ".png";
	rs2::save_to_png(filename_png.data(), frame.get_width(), frame.get_height(), frame.get_bytes_per_pixel(),
		frame.get_data(), frame.get_width() * frame.get_bytes_per_pixel());

}