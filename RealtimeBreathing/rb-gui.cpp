/* Realtime Breathing Gui App */
/* To use multiple cameras, see rs-multicam: It's possible to use a pipe the same way, and measure all frames in all pipes. */

#include <librealsense2/rs.hpp>
#include "example.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include <iostream>
#include "rb_aux.h"
#include <opencv2/opencv.hpp>

// copied os.h because project does comile when including it (seems to be due to double inclusion of rendering.h)
// *****	START of os.h copy	*****

//#pragma once
//#include <vector>
//#include <string>
//#include <rendering.h>
//struct GLFWmonitor;
//struct GLFWwindow;

namespace rs2
{
	// Wrapper for cross-platform dialog control
	enum file_dialog_mode {
		open_file = (1 << 0),
		save_file = (1 << 1),
	};
	const char* file_dialog_open(file_dialog_mode flags, const char* filters, const char* default_path, const char* default_name);
}

// *****	END of os.h copy	*****

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

	const char* filename = nullptr;	// filename will hold the name of an existing file chosen bu user to analyze.
									// defined here, so that nullity can indeicate if file was already chosen or not.
	bool run_on_existing_file = false; //When true, run analysis for an existing file chosen by user through open_dialog
	bool recording = false; //When true, record camera stream to file

	while (app) // application still alive?
	{

		// Flags for displaying ImGui window
		static const int flags = ImGuiWindowFlags_NoCollapse
			| ImGuiWindowFlags_NoSavedSettings
			//	| ImGuiWindowFlags_NoResize
			| ImGuiWindowFlags_AlwaysAutoResize;

		// render the ui:
		ImGui_ImplGlfw_NewFrame(1);
		//ImGui::NewFrame();

		ImGui::Begin("Menu", nullptr, flags); // Create a window called "Menu" and append into it
		ImGui::Checkbox("Show Camera", &show_camera_stream);      // Checkbox: showing the camera stream
		ImGui::Checkbox("Choose existing file", &run_on_existing_file);      // Checkbox: Choose an existing file to play and run anlysis for
		//ImGui::End();

		//if (show_camera_stream) {

			//if (!stream_enabled) {
			//	cfg.enable_stream(RS2_STREAM_DEPTH);
			//	cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
			//	pipe.start(cfg);

			//	stream_enabled = true;
			//}

			
			if (show_camera_stream && !run_on_existing_file) {
			
				if (filename) {
					/*
					if run on file was unchecked, make sure pipe is stopped
					*/
					cfg.disable_all_streams();
					cfg = rs2::config();
					pipe.stop();
					//reset filename argumenr, so that if 'choose existing file' is clicked again, a new explorer window will appear
					filename = nullptr;
				}

				if (!stream_enabled) {
					cfg.enable_stream(RS2_STREAM_DEPTH);
					cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
					pipe.start(cfg);
					stream_enabled = true;
				}

				if (ImGui::Button("record", { 50, 50 }))
				{
					pipe.stop(); // Stop the pipeline with the default configuration
					const char* out_filename = rs2::file_dialog_open(rs2::file_dialog_mode::save_file, "ROS-bag\0*.bag\0", NULL, NULL);
					cfg.enable_record_to_file(out_filename);
					pipe.start(cfg); //File will be opened at this point
					recording = true;
				
				}
				if (recording) {
					if (ImGui::Button("stop\nrecord", { 50, 50 }))
					{
						cfg.disable_all_streams();
						cfg = rs2::config();
						pipe.stop(); // Stop the pipeline that holds the file and the recorder
						pipe.start(cfg);
						recording = false;
					}
				}
				
				

			}
			if (!show_camera_stream && run_on_existing_file) {
				if (stream_enabled) {
					/*
					if "show camera" was unchecked, make sure stream is disabled
					*/
					cfg.disable_stream(RS2_STREAM_DEPTH);
					cfg.disable_stream(RS2_STREAM_COLOR);
					pipe.stop();
					stream_enabled = false;
				}

				if (!filename) {
					filename = rs2::file_dialog_open(rs2::file_dialog_mode::open_file, "ROS-bag\0*.bag\0", NULL, NULL);
					cfg.enable_device_from_file(filename);
					pipe.start(cfg); //File will be opened in read mode at this point

				}
			}
			if (!show_camera_stream && !run_on_existing_file) {
				if (filename) {
					/*
					if run on file was unchecked, make sure pipe is stopped
					*/
					cfg.disable_all_streams();
					cfg = rs2::config();
					pipe.stop();
					//reset filename argumenr, so that if 'choose existing file' is clicked again, a new explorer window will appear
					filename = nullptr;
				}

				if (stream_enabled) {
					/*
					if "show camera" was unchecked, make sure stream is disabled
					*/
					cfg.disable_stream(RS2_STREAM_DEPTH);
					cfg.disable_stream(RS2_STREAM_COLOR);
					pipe.stop();
					stream_enabled = false;
				}
				ImGui::End();
				ImGui::Render();
				continue;
			}

			if (show_camera_stream && run_on_existing_file) {
				ImGui::End();
				ImGui::Render();
				continue;
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

			frame_manager.process_frame(color, depth);
			

			// convert the newly-arrived frames to render-firendly format
			//for (const auto& frame : fs) //iterate over all available frames. removed to ignore IR emmitter frames.
			//{
				render_frames[color.get_profile().unique_id()] = colorizer.process(color);
				render_frames[depth.get_profile().unique_id()] = colorizer.process(depth);
			//}

			// present all the collected frames with opengl mosaic
			app.show(render_frames);


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
			//ImGui::End();

		//} else { //stop camera:
		//	if (stream_enabled) {
		//		cfg.disable_stream(RS2_STREAM_DEPTH);
		//		cfg.disable_stream(RS2_STREAM_COLOR);
		//		pipe.stop();
		//		stream_enabled = false;
		//	}
		//	ImGui::Render();
		//	continue;
		//}
		
		ImGui::End();
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
