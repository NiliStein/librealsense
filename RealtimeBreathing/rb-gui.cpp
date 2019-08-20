/* Realtime Breathing Gui */

#include <librealsense2/rs.hpp>
#include "example.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"

// This will require several standard data-structures and algorithms:
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>

int main(int argc, char * argv[]) try
{
	// Create and initialize GUI related objects
	window app(1280, 720, "RealtimeBreathing"); // Create windows app window
	ImGui_ImplGlfw_Init(app, false);      // ImGui library intializition
	rs2::colorizer colorizer;		// Helper to colorize depth images
	texture depth_image, color_image;     // Helpers for rendering images

	// Create a pipeline to easily configure and start the camera
	rs2::pipeline pipe;
	rs2::config cfg;

	// Define two align objects. One will be used to align
	// to depth viewport and the other to color.
	// Creating align object is an expensive operation
	// that should not be performed in the main loop
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	rs2::align align_to_color(RS2_STREAM_COLOR);

	bool camera_on = false;

	while (app) // Application still alive?
	{
		// Render the UI:
		ImGui_ImplGlfw_NewFrame(1);

		if (ImGui::Button("Start_Camera") || camera_on) {

			if (!camera_on) {
				cfg.enable_stream(RS2_STREAM_DEPTH);
				cfg.enable_stream(RS2_STREAM_COLOR);
				pipe.start(cfg);
				camera_on = true;
			}

			if (ImGui::Button("Stop_Camera")) {
				camera_on = false;
				cfg.disable_stream(RS2_STREAM_DEPTH);
				cfg.disable_stream(RS2_STREAM_COLOR);
				pipe.stop();
				continue;
			}

			// Using the align object, we block the application until a frameset is available
			rs2::frameset frameset_depth = pipe.wait_for_frames();
			rs2::frameset frameset_color = pipe.wait_for_frames();

			// Align all frames to depth viewport
			frameset_depth = align_to_depth.process(frameset_depth);

			// Align all frames to color viewport
			frameset_color = align_to_color.process(frameset_color);

			// With the aligned frameset we proceed as usual
			auto depth = frameset_depth.get_depth_frame();
			auto color = frameset_color.get_color_frame();
			auto colorized_depth = colorizer.colorize(depth);

			//TODO: split to 2 windows depth and color
			depth_image.render(colorized_depth, { 640, 360, app.width()/2, app.height()/2 });

			color_image.render(color, { 0, 360, app.width() / 2, app.height() / 2 });

			glColor4f(1.f, 1.f, 1.f, 1.f);
			glDisable(GL_BLEND);

		}

		ImGui::Render();
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	////If no camera found, we don't want to fail the app:
	//if (!strcmp(e.what(), "No device connected")) {
	//	ImGui::OpenPopup("Camera Disconnected");
	//	bool open = true;
	//	if (ImGui::BeginPopupModal("Camera Disconnected", &open))
	//	{
	//		//ImGui::Text("Camera is disconnected!\nConnect camera and then click 'Retry'.");
	//		//if (ImGui::Button("Retry")) {
	//		//	ImGui::CloseCurrentPopup();
	//		//}
	//		//ImGui::EndPopup();
	//	}
	//}
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception & e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}
