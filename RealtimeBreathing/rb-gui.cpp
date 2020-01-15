/* Realtime Breathing Gui App */
/* To use multiple cameras, see rs-multicam: It's possible to use a pipe the same way, and measure all frames in all pipes. */

#include <librealsense2/rs.hpp>
#include "example.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include <iostream>
#include "rb_aux.h"
#include <opencv2/opencv.hpp>

int main(int argc, char * argv[]) try
{
	window app(1280, 720, "RealtimeBreathing");

	//ImGui::CreateContext();

	ImGui_ImplGlfw_Init(app, false);      // imgui library intializition
	rs2::colorizer colorizer;		// helper to colorize depth images
	texture depth_image, color_image;     // helpers for rendering images

	// create a pipeline to easily configure and start the camera
	rs2::pipeline pipe;
	rs2::config cfg;

	// define two align objects. one will be used to align
	// to depth viewport and the other to color.
	// creating align object is an expensive operation
	// that should not be performed in the main loop
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	rs2::align align_to_color(RS2_STREAM_COLOR);

	FrameManager frame_manager;

	bool show_camera_stream = false;
	bool stream_enabled = false;
	bool start_camera = false;

	while (app) // application still alive?
	{

		// render the ui:
		ImGui_ImplGlfw_NewFrame(1);
		//ImGui::NewFrame();

		ImGui::Begin("Menu"); // Create a window called "Menu" and append into it
		ImGui::Checkbox("Show Camera", &show_camera_stream);      // Checkbox: showing the camera stream
		ImGui::End();
		if (show_camera_stream) {

			ImGui::Begin("Stream");

			if (!stream_enabled) {
				cfg.enable_stream(RS2_STREAM_DEPTH);
				cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
				pipe.start(cfg);

				stream_enabled = true;
			}

			// using the align object, we block the application until a frameset is available
			rs2::frameset fs = pipe.wait_for_frames();
			rs2::frameset frameset_depth(fs);
			rs2::frameset frameset_color(fs);

			// align all frames to depth viewport
			frameset_depth = align_to_depth.process(frameset_depth);

			// align all frames to color viewport
			frameset_color = align_to_color.process(frameset_color);

			// with the aligned frameset we proceed as usual
			auto depth = frameset_depth.get_depth_frame();
			auto color = frameset_color.get_color_frame();
			auto colorized_depth = colorizer.colorize(depth);
			
			//collect all frames:
			//using a map as in rs-multicam to allow future changes in number of cameras displayed.
			std::map<int, rs2::frame> render_frames;

			//for (const rs2::frame& f : frameset_color) {
			//	
			//	//save_last_frame("frames\\frame", f);
			//	// TODO: Currently, the format of the frame data is only RS2_FORMAT_RGB8, RS2_FORMAT_BGR8 causes exception....
			//	const void * color_frame_data = f.get_data();
			//	cv::Mat rgb8_mat(cv::Size(color_frame_width, color_frame_height), CV_8UC3, (void *)color_frame_data, cv::Mat::AUTO_STEP);
			//	cv::Mat bgr8_mat(cv::Size(color_frame_width, color_frame_height), CV_8UC3);
			//	cv::cvtColor(rgb8_mat, bgr8_mat, cv::COLOR_RGB2BGR);
			//	//new_frames.emplace_back(f);
			//}

			//TODO: 3d distances
			frame_manager.process_color_frame(color);

			// convert the newly-arrived frames to render-firendly format
			for (const auto& frame : fs)
			{
				render_frames[frame.get_profile().unique_id()] = colorizer.process(frame);
			}

			// present all the collected frames with opengl mosaic
			app.show(render_frames);

			

			//the following commented code was the previous implementation to the division to two images of depth and color:
			////two split frames in app, left for color and right for depth:
			//depth_image.render(colorized_depth, { 640, 360, app.width() / 2, app.height() / 2 });
			//color_image.render(color, { 0, 360, app.width() / 2, app.height() / 2 });

			glColor4f(1.f, 1.f, 1.f, 1.f);
			glDisable(GL_BLEND);
#if 0			 
			//todo: fix distance presentation
			//show the distance of the image from the camera:
			// try to get a frame of a depth image
			rs2::depth_frame depth_frame = frameset_depth.get_depth_frame();

			// get the depth frame's dimensions
			float depth_frame_width = depth_frame.get_width();
			float depth_frame_height = depth_frame.get_height();

			// query the distance from the camera to the object in the center of the image
			float dist_to_center = depth.get_distance(depth_frame_width / 2, depth_frame_height / 2);

			//show the distance in a dialog box:
			bool open = true;
			if (ImGui::BeginPopupModal("distance from camera", &open))
			{
				//the following function uses printf() format string:
				ImGui::TextWrapped("distance from camera is: %f", dist_to_center);
			}
#endif
			ImGui::End();

		} else { //stop camera:
			if (stream_enabled) {
				cfg.disable_stream(RS2_STREAM_DEPTH);
				cfg.disable_stream(RS2_STREAM_COLOR);
				pipe.stop();
				stream_enabled = false;
			}
			ImGui::Render();
			continue;
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
