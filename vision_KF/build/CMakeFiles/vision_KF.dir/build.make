# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Desktop/vision_KF

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Desktop/vision_KF/build

# Include any dependencies generated for this target.
include CMakeFiles/vision_KF.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/vision_KF.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/vision_KF.dir/flags.make

CMakeFiles/vision_KF.dir/src/main.cpp.o: CMakeFiles/vision_KF.dir/flags.make
CMakeFiles/vision_KF.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/vision_KF/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/vision_KF.dir/src/main.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vision_KF.dir/src/main.cpp.o -c /home/pi/Desktop/vision_KF/src/main.cpp

CMakeFiles/vision_KF.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vision_KF.dir/src/main.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/vision_KF/src/main.cpp > CMakeFiles/vision_KF.dir/src/main.cpp.i

CMakeFiles/vision_KF.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vision_KF.dir/src/main.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/vision_KF/src/main.cpp -o CMakeFiles/vision_KF.dir/src/main.cpp.s

CMakeFiles/vision_KF.dir/src/Kalman.cpp.o: CMakeFiles/vision_KF.dir/flags.make
CMakeFiles/vision_KF.dir/src/Kalman.cpp.o: ../src/Kalman.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Desktop/vision_KF/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/vision_KF.dir/src/Kalman.cpp.o"
	/usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vision_KF.dir/src/Kalman.cpp.o -c /home/pi/Desktop/vision_KF/src/Kalman.cpp

CMakeFiles/vision_KF.dir/src/Kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vision_KF.dir/src/Kalman.cpp.i"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Desktop/vision_KF/src/Kalman.cpp > CMakeFiles/vision_KF.dir/src/Kalman.cpp.i

CMakeFiles/vision_KF.dir/src/Kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vision_KF.dir/src/Kalman.cpp.s"
	/usr/bin/arm-linux-gnueabihf-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Desktop/vision_KF/src/Kalman.cpp -o CMakeFiles/vision_KF.dir/src/Kalman.cpp.s

# Object files for target vision_KF
vision_KF_OBJECTS = \
"CMakeFiles/vision_KF.dir/src/main.cpp.o" \
"CMakeFiles/vision_KF.dir/src/Kalman.cpp.o"

# External object files for target vision_KF
vision_KF_EXTERNAL_OBJECTS =

vision_KF: CMakeFiles/vision_KF.dir/src/main.cpp.o
vision_KF: CMakeFiles/vision_KF.dir/src/Kalman.cpp.o
vision_KF: CMakeFiles/vision_KF.dir/build.make
vision_KF: /usr/local/lib/libopencv_gapi.so.4.5.0
vision_KF: /usr/local/lib/libopencv_stitching.so.4.5.0
vision_KF: /usr/local/lib/libopencv_aruco.so.4.5.0
vision_KF: /usr/local/lib/libopencv_bgsegm.so.4.5.0
vision_KF: /usr/local/lib/libopencv_bioinspired.so.4.5.0
vision_KF: /usr/local/lib/libopencv_ccalib.so.4.5.0
vision_KF: /usr/local/lib/libopencv_dnn_objdetect.so.4.5.0
vision_KF: /usr/local/lib/libopencv_dnn_superres.so.4.5.0
vision_KF: /usr/local/lib/libopencv_dpm.so.4.5.0
vision_KF: /usr/local/lib/libopencv_face.so.4.5.0
vision_KF: /usr/local/lib/libopencv_freetype.so.4.5.0
vision_KF: /usr/local/lib/libopencv_fuzzy.so.4.5.0
vision_KF: /usr/local/lib/libopencv_hdf.so.4.5.0
vision_KF: /usr/local/lib/libopencv_hfs.so.4.5.0
vision_KF: /usr/local/lib/libopencv_img_hash.so.4.5.0
vision_KF: /usr/local/lib/libopencv_intensity_transform.so.4.5.0
vision_KF: /usr/local/lib/libopencv_line_descriptor.so.4.5.0
vision_KF: /usr/local/lib/libopencv_mcc.so.4.5.0
vision_KF: /usr/local/lib/libopencv_quality.so.4.5.0
vision_KF: /usr/local/lib/libopencv_rapid.so.4.5.0
vision_KF: /usr/local/lib/libopencv_reg.so.4.5.0
vision_KF: /usr/local/lib/libopencv_rgbd.so.4.5.0
vision_KF: /usr/local/lib/libopencv_saliency.so.4.5.0
vision_KF: /usr/local/lib/libopencv_stereo.so.4.5.0
vision_KF: /usr/local/lib/libopencv_structured_light.so.4.5.0
vision_KF: /usr/local/lib/libopencv_superres.so.4.5.0
vision_KF: /usr/local/lib/libopencv_surface_matching.so.4.5.0
vision_KF: /usr/local/lib/libopencv_tracking.so.4.5.0
vision_KF: /usr/local/lib/libopencv_videostab.so.4.5.0
vision_KF: /usr/local/lib/libopencv_xfeatures2d.so.4.5.0
vision_KF: /usr/local/lib/libopencv_xobjdetect.so.4.5.0
vision_KF: /usr/local/lib/libopencv_xphoto.so.4.5.0
vision_KF: /usr/lib/libwiringPi.so
vision_KF: /usr/local/lib/libopencv_shape.so.4.5.0
vision_KF: /usr/local/lib/libopencv_highgui.so.4.5.0
vision_KF: /usr/local/lib/libopencv_datasets.so.4.5.0
vision_KF: /usr/local/lib/libopencv_plot.so.4.5.0
vision_KF: /usr/local/lib/libopencv_text.so.4.5.0
vision_KF: /usr/local/lib/libopencv_dnn.so.4.5.0
vision_KF: /usr/local/lib/libopencv_ml.so.4.5.0
vision_KF: /usr/local/lib/libopencv_phase_unwrapping.so.4.5.0
vision_KF: /usr/local/lib/libopencv_optflow.so.4.5.0
vision_KF: /usr/local/lib/libopencv_ximgproc.so.4.5.0
vision_KF: /usr/local/lib/libopencv_video.so.4.5.0
vision_KF: /usr/local/lib/libopencv_videoio.so.4.5.0
vision_KF: /usr/local/lib/libopencv_imgcodecs.so.4.5.0
vision_KF: /usr/local/lib/libopencv_objdetect.so.4.5.0
vision_KF: /usr/local/lib/libopencv_calib3d.so.4.5.0
vision_KF: /usr/local/lib/libopencv_features2d.so.4.5.0
vision_KF: /usr/local/lib/libopencv_flann.so.4.5.0
vision_KF: /usr/local/lib/libopencv_photo.so.4.5.0
vision_KF: /usr/local/lib/libopencv_imgproc.so.4.5.0
vision_KF: /usr/local/lib/libopencv_core.so.4.5.0
vision_KF: CMakeFiles/vision_KF.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Desktop/vision_KF/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable vision_KF"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vision_KF.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/vision_KF.dir/build: vision_KF

.PHONY : CMakeFiles/vision_KF.dir/build

CMakeFiles/vision_KF.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/vision_KF.dir/cmake_clean.cmake
.PHONY : CMakeFiles/vision_KF.dir/clean

CMakeFiles/vision_KF.dir/depend:
	cd /home/pi/Desktop/vision_KF/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Desktop/vision_KF /home/pi/Desktop/vision_KF /home/pi/Desktop/vision_KF/build /home/pi/Desktop/vision_KF/build /home/pi/Desktop/vision_KF/build/CMakeFiles/vision_KF.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/vision_KF.dir/depend

