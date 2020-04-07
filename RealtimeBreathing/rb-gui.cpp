/* Realtime Breathing Gui App */
/* To use multiple cameras, see rs-multicam: It's possible to use a pipe the same way, and measure all frames in all pipes. */

#include <librealsense2/rs.hpp>
#include "example.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include <iostream>
#include "rb_aux.h"
#include <opencv2/opencv.hpp>
#include <chrono>

// copied os.h because project does comile when including it (seems to be due to double inclusion of rendering.h)
// *****	START of os.h copy	*****

//#pragma once
//#include <vector>
//#include <string>
//#include <rendering.h>
//struct GLFWmonitor;
//struct GLFWwindow;

#define FILE_ON_REPEAT false
extern void init_logFile(const char* filename, int num_of_stickers, std::string D2units);
extern std::ofstream logFile;
extern bool CALC_2D_BY_CM;
extern int NUM_OF_STICKERS;
const char* config_err1 = "Warning: distance from mid1 was set to y, while number of stickers is 4. This distance will be disregarded.";
const char* config_err2 = "Warning: location of mid1 was set to y, while number of stickers is 4. This location will be disregarded.";
const char* config_errors[2] = { config_err1, config_err2 };
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


	int config_res;
	Config user_cfg(CONFIG_FILEPATH, &config_res);
	FrameManager frame_manager(&user_cfg);
	GraphPlot graph(user_cfg.mode, user_cfg.dimension, frame_manager.manager_start_time);

	bool show_camera_stream = false;
	bool stream_enabled = false;
	bool start_camera = false;

	const char* filename = nullptr;	// filename will hold the name of an existing file chosen bu user to analyze.
									// defined here, so that nullity can indeicate if file was already chosen or not.
	bool run_on_existing_file = false; // When true, run analysis for an existing file chosen by user through open_dialog.
	bool recording = false; // When true, record camera stream to file.
	bool pause = false;	// when true, pause streaming file. only available while sreaming from an existing file.
	clock_t start_time, end_time; // measure time, for 15 seconds intervals.
	long double f = 0; 
	long double bpm = 0;

	//&&&&&&&& moved declaration for within the main loop to here, for use of freezing frame when pausing a stream from file
	rs2::frameset fs; 
	while (app) // application still alive?
	{
		// Flags for displaying ImGui window
		static const int flags = ImGuiWindowFlags_NoCollapse
			| ImGuiWindowFlags_NoSavedSettings
			| ImGuiWindowFlags_AlwaysAutoResize;

		// render the ui:
		ImGui_ImplGlfw_NewFrame(1);

		ImGui::Begin("Menu", nullptr, flags); // Create a window called "Menu" and append into it
		ImGui::Checkbox("Show Camera", &show_camera_stream);      // Checkbox: showing the camera stream
		ImGui::Checkbox("Choose existing file", &run_on_existing_file);      // Checkbox: Choose an existing file to play and run anlysis for
		if (user_cfg.mode != graph_mode::LOCATION) ImGui::Text("Frequency: %f	BPM:  %f", f, bpm);
		if (config_res) ImGui::Text(config_errors[config_res - 1]);
		
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
					logFile.close();
				}

				if (!stream_enabled) {
					cfg.enable_stream(RS2_STREAM_DEPTH);
					cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
					pipe.start(cfg);
					stream_enabled = true;
					start_time = clock();
					frame_manager.reset(); // reset FrameManager for additional processing
					graph.reset(frame_manager.manager_start_time);
					std::string D2units = (CALC_2D_BY_CM) ? "cm" : "pixels";
					init_logFile(filename, NUM_OF_STICKERS, D2units);

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
					logFile.close();
				}

				if (!filename) {
					filename = rs2::file_dialog_open(rs2::file_dialog_mode::open_file, "ROS-bag\0*.bag\0", NULL, NULL);
					if (filename) {
						cfg.enable_device_from_file(filename, FILE_ON_REPEAT);
						start_time = clock();
						pipe.start(cfg); //File will be opened in read mode at this point
						frame_manager.reset(); // reset FrameManager for additional processing
						graph.reset(frame_manager.manager_start_time);
						std::string D2units = (CALC_2D_BY_CM) ? "cm" : "pixels";
						init_logFile(filename, NUM_OF_STICKERS, D2units);
					}
					else { //user clicked -choose file- ans then clicked -cancel-
						run_on_existing_file = false;
					}
					
				}

				if (pause) {
					if (ImGui::Button("unpause", { 50, 50 })) {
						pause = false;
						rs2::device device = pipe.get_active_profile().get_device();
						rs2::playback playback = device.as<rs2::playback>();
						playback.resume();
					}
					else {
						//&&&&&&&& following code freezez the last frame while pausing
						if (fs.size() > 0) {

							auto d = fs.get_depth_frame();
							auto c = fs.get_color_frame();
							
							std::map<int, rs2::frame> freeze_frames;
							freeze_frames[0] = colorizer.process(c);
							freeze_frames[1] = colorizer.process(d);

							// present last collected frame
							app.show(freeze_frames);
						}
						ImGui::End();
						ImGui::Render();
						continue;
					}
				}
				if (!pause) {
					if (ImGui::Button("pause", { 50, 50 }))
					{
						pause = true;
						rs2::device device = pipe.get_active_profile().get_device();
						rs2::playback playback = device.as<rs2::playback>();
						playback.pause();
						ImGui::End();
						ImGui::Render();
						continue;
					}

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
					//reset filename argument, so that if 'choose existing file' is clicked again, a new explorer window will appear
					filename = nullptr;
					logFile.close();
				}

				if (stream_enabled) {
					/*
					if "show camera" was unchecked, make sure stream is disabled
					*/
					cfg.disable_stream(RS2_STREAM_DEPTH);
					cfg.disable_stream(RS2_STREAM_COLOR);
					pipe.stop();
					stream_enabled = false;
					logFile.close();
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
			//&&&&&&&& moved declaration to out of main loop (above)
			//rs2::frameset fs; 
			if (run_on_existing_file && !FILE_ON_REPEAT) {
				if (!pipe.try_wait_for_frames(&fs, 1000)) {
					/*
					if run on file ended, stop pipe
					*/
					run_on_existing_file = false;
					cfg.disable_all_streams();
					cfg = rs2::config();
					pipe.stop();
					//reset filename argument, so that if 'choose existing file' is clicked again, a new explorer window will appear
					filename = nullptr;
					logFile.close();
					continue;
				}
			}
			else {
				fs = pipe.wait_for_frames();
			}

		/*	rs2::frameset frameset_depth(fs);
			rs2::frameset frameset_color(fs);*/

			// align all frames to depth viewport
			//fs = align_to_depth.process(fs);

			// align all frames to color viewport
			fs = align_to_color.process(fs);
			// with the aligned frameset we proceed as usual
			auto depth = fs.get_depth_frame();
			auto color = fs.get_color_frame();
			auto colorized_depth = colorizer.colorize(depth);
			
			//collect all frames:
			//using a map as in rs-multicam to allow future changes in number of cameras displayed.
			std::map<int, rs2::frame> render_frames;

			frame_manager.process_frame(color, depth);

			// convert the newly-arrived frames to render-firendly format
			//for (const auto& frame : fs) //iterate over all available frames. removed to ignore IR emitter frames.
			//{
				render_frames[color.get_profile().unique_id()] = colorizer.process(color);
				render_frames[depth.get_profile().unique_id()] = colorizer.process(depth);
			//}

			// present all the collected frames with opengl mosaic
			app.show(render_frames);
			
			end_time = clock();
			//set interval of 15 seconds:
			if (double(end_time - start_time) / double(CLOCKS_PER_SEC) >= 15.0) {
				frame_manager.activateInterval(); //activate calculation since we have 15 seconds at least
			}

			ImGui::NextColumn();

			
			if (user_cfg.mode == graph_mode::DISTANCES) {

				std::vector<cv::Point2d> points;
				frame_manager.get_dists(&points);
				graph.plot(points);
				
			}
			if (user_cfg.mode == graph_mode::FOURIER) {
				std::vector<cv::Point2d> points;
				frame_manager.get_dists(&points);
				graph.plot(points);
			}
			if (user_cfg.mode == graph_mode::LOCATION) {
				const char * lineSpec[5] = { "-k", "-g", "-b", "-r", "-y" };
				std::vector<cv::Point2d> points[5];
				for (int stInt = stickers::left; stInt != stickers::sdummy; stInt++) {
					stickers s = static_cast<stickers>(stInt);
					if (frame_manager.user_cfg->stickers_included[s]) {
						frame_manager.get_locations(s, &points[stInt]);
						graph.plot(points[stInt], lineSpec[stInt]);
					}
				}
			}
			if (user_cfg.mode == graph_mode::NOGRAPH) {
				std::vector<cv::Point2d> points;
				frame_manager.get_dists(&points);
				graph.plot(points);
			}
			
			glColor4f(1.f, 1.f, 1.f, 1.f);
			glDisable(GL_BLEND);
		
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
