# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tobor/data-beagleboard/bb_navigation/data_robot

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tobor/data-beagleboard/bb_navigation/data_robot/build

# Include any dependencies generated for this target.
include src/application_services/CMakeFiles/data_robot_application_services.dir/depend.make

# Include the progress variables for this target.
include src/application_services/CMakeFiles/data_robot_application_services.dir/progress.make

# Include the compile flags for this target's objects.
include src/application_services/CMakeFiles/data_robot_application_services.dir/flags.make

src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o: src/application_services/CMakeFiles/data_robot_application_services.dir/flags.make
src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o: ../src/application_services/EncoderCountsReportingService.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tobor/data-beagleboard/bb_navigation/data_robot/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o"
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o -c /home/tobor/data-beagleboard/bb_navigation/data_robot/src/application_services/EncoderCountsReportingService.cpp

src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.i"
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tobor/data-beagleboard/bb_navigation/data_robot/src/application_services/EncoderCountsReportingService.cpp > CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.i

src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.s"
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tobor/data-beagleboard/bb_navigation/data_robot/src/application_services/EncoderCountsReportingService.cpp -o CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.s

src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o.requires:
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o.requires

src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o.provides: src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o.requires
	$(MAKE) -f src/application_services/CMakeFiles/data_robot_application_services.dir/build.make src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o.provides.build
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o.provides

src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o.provides.build: src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o.provides.build

src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o: src/application_services/CMakeFiles/data_robot_application_services.dir/flags.make
src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o: ../src/application_services/TickVelocityCommandService.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tobor/data-beagleboard/bb_navigation/data_robot/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o"
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o -c /home/tobor/data-beagleboard/bb_navigation/data_robot/src/application_services/TickVelocityCommandService.cpp

src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.i"
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tobor/data-beagleboard/bb_navigation/data_robot/src/application_services/TickVelocityCommandService.cpp > CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.i

src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.s"
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tobor/data-beagleboard/bb_navigation/data_robot/src/application_services/TickVelocityCommandService.cpp -o CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.s

src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o.requires:
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o.requires

src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o.provides: src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o.requires
	$(MAKE) -f src/application_services/CMakeFiles/data_robot_application_services.dir/build.make src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o.provides.build
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o.provides

src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o.provides.build: src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o.provides.build

# Object files for target data_robot_application_services
data_robot_application_services_OBJECTS = \
"CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o" \
"CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o"

# External object files for target data_robot_application_services
data_robot_application_services_EXTERNAL_OBJECTS =

../lib/libdata_robot_application_services.a: src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o
../lib/libdata_robot_application_services.a: src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o
../lib/libdata_robot_application_services.a: src/application_services/CMakeFiles/data_robot_application_services.dir/build.make
../lib/libdata_robot_application_services.a: src/application_services/CMakeFiles/data_robot_application_services.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ../../../lib/libdata_robot_application_services.a"
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services && $(CMAKE_COMMAND) -P CMakeFiles/data_robot_application_services.dir/cmake_clean_target.cmake
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/data_robot_application_services.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/application_services/CMakeFiles/data_robot_application_services.dir/build: ../lib/libdata_robot_application_services.a
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/build

src/application_services/CMakeFiles/data_robot_application_services.dir/requires: src/application_services/CMakeFiles/data_robot_application_services.dir/EncoderCountsReportingService.o.requires
src/application_services/CMakeFiles/data_robot_application_services.dir/requires: src/application_services/CMakeFiles/data_robot_application_services.dir/TickVelocityCommandService.o.requires
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/requires

src/application_services/CMakeFiles/data_robot_application_services.dir/clean:
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services && $(CMAKE_COMMAND) -P CMakeFiles/data_robot_application_services.dir/cmake_clean.cmake
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/clean

src/application_services/CMakeFiles/data_robot_application_services.dir/depend:
	cd /home/tobor/data-beagleboard/bb_navigation/data_robot/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tobor/data-beagleboard/bb_navigation/data_robot /home/tobor/data-beagleboard/bb_navigation/data_robot/src/application_services /home/tobor/data-beagleboard/bb_navigation/data_robot/build /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services /home/tobor/data-beagleboard/bb_navigation/data_robot/build/src/application_services/CMakeFiles/data_robot_application_services.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/application_services/CMakeFiles/data_robot_application_services.dir/depend

