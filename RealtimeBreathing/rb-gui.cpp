/* Realtime Breathing Gui App */
/* To use multiple cameras, see rs-multicam: It's possible to use a pipe the same way, and measure all frames in all pipes. */

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
	//rs2::log_to_console(RS2_LOG_SEVERITY_WARN);
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

		//Stat camera botton:
		if (ImGui::Button("Start_Camera") || camera_on) {

			if (!camera_on) {
				cfg.enable_stream(RS2_STREAM_DEPTH);
				cfg.enable_stream(RS2_STREAM_COLOR);
				pipe.start(cfg);
				camera_on = true;
			}
			
			//Stop camera toggle button:
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

			//Collect all frames:
			//Using a map as in rs-multicam to allow future changes in number of cameras displayed.
			std::map<int, rs2::frame> render_frames;
			std::vector<rs2::frame> new_frames;
			rs2::frameset fs;
			if (pipe.poll_for_frames(&fs))
			{
				for (const rs2::frame& f : fs)
					new_frames.emplace_back(f);
			}

			// Convert the newly-arrived frames to render-firendly format
			for (const auto& frame : new_frames)
			{
				render_frames[frame.get_profile().unique_id()] = colorizer.process(frame);
			}

			// Present all the collected frames with openGl mosaic
			app.show(render_frames);

			//The following commented code was the previous implementation to the division to two images of depth and color:
			////Two split frames in app, left for color and right for depth:
			//depth_image.render(colorized_depth, { 640, 360, app.width() / 2, app.height() / 2 });
			//color_image.render(color, { 0, 360, app.width() / 2, app.height() / 2 });

			glColor4f(1.f, 1.f, 1.f, 1.f);
			glDisable(GL_BLEND);

			//TODO: Fix distance presentation
			//Show the distance of the image from the camera:
			// Try to get a frame of a depth image
			rs2::depth_frame depth_frame = frameset_depth.get_depth_frame();

			// Get the depth frame's dimensions
			float depth_frame_width = depth_frame.get_width();
			float depth_frame_height = depth_frame.get_height();

			// Query the distance from the camera to the object in the center of the image
			float dist_to_center = depth.get_distance(depth_frame_width / 2, depth_frame_height / 2);

			//Show the distance in a dialog box:
			bool open = true;
			if (ImGui::BeginPopupModal("Distance From Camera", &open))
			{
				//The following function uses printf() format string:
				ImGui::TextWrapped("Distance From Camera is: %f", dist_to_center);
			}
			ImGui::Render();
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
