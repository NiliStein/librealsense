#pragma once

#include <librealsense2/rs.hpp>

void save_last_frame(const char* filename, const rs2::video_frame& frame);

#define NUM_OF_LAST_FRAMES 10