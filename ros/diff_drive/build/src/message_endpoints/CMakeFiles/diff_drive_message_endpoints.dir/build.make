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
CMAKE_SOURCE_DIR = /home/tobor/data-beagleboard/bb_navigation/diff_drive

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tobor/data-beagleboard/bb_navigation/diff_drive/build

# Include any dependencies generated for this target.
include src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/depend.make

# Include the progress variables for this target.
include src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/progress.make

# Include the compile flags for this target's objects.
include src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/flags.make

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/flags.make
src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o: ../src/message_endpoints/EncoderCountsEndpoint.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o -c /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/EncoderCountsEndpoint.cpp

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.i"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/EncoderCountsEndpoint.cpp > CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.i

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.s"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/EncoderCountsEndpoint.cpp -o CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.s

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o.requires:
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o.requires

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o.provides: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o.requires
	$(MAKE) -f src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/build.make src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o.provides.build
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o.provides

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o.provides.build: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o.provides.build

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/flags.make
src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o: ../src/message_endpoints/OdometryEndpoint.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o -c /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/OdometryEndpoint.cpp

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.i"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/OdometryEndpoint.cpp > CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.i

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.s"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/OdometryEndpoint.cpp -o CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.s

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o.requires:
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o.requires

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o.provides: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o.requires
	$(MAKE) -f src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/build.make src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o.provides.build
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o.provides

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o.provides.build: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o.provides.build

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/flags.make
src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o: ../src/message_endpoints/TickVelocityEndpoint.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o -c /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/TickVelocityEndpoint.cpp

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.i"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/TickVelocityEndpoint.cpp > CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.i

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.s"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/TickVelocityEndpoint.cpp -o CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.s

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o.requires:
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o.requires

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o.provides: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o.requires
	$(MAKE) -f src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/build.make src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o.provides.build
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o.provides

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o.provides.build: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o.provides.build

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/flags.make
src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o: ../src/message_endpoints/TwistEndpoint.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o -c /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/TwistEndpoint.cpp

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.i"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/TwistEndpoint.cpp > CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.i

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.s"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints/TwistEndpoint.cpp -o CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.s

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o.requires:
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o.requires

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o.provides: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o.requires
	$(MAKE) -f src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/build.make src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o.provides.build
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o.provides

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o.provides.build: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o.provides.build

# Object files for target diff_drive_message_endpoints
diff_drive_message_endpoints_OBJECTS = \
"CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o" \
"CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o" \
"CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o" \
"CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o"

# External object files for target diff_drive_message_endpoints
diff_drive_message_endpoints_EXTERNAL_OBJECTS =

../lib/libdiff_drive_message_endpoints.a: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o
../lib/libdiff_drive_message_endpoints.a: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o
../lib/libdiff_drive_message_endpoints.a: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o
../lib/libdiff_drive_message_endpoints.a: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o
../lib/libdiff_drive_message_endpoints.a: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/build.make
../lib/libdiff_drive_message_endpoints.a: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ../../../lib/libdiff_drive_message_endpoints.a"
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && $(CMAKE_COMMAND) -P CMakeFiles/diff_drive_message_endpoints.dir/cmake_clean_target.cmake
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/diff_drive_message_endpoints.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/build: ../lib/libdiff_drive_message_endpoints.a
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/build

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/requires: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/EncoderCountsEndpoint.o.requires
src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/requires: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/OdometryEndpoint.o.requires
src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/requires: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TickVelocityEndpoint.o.requires
src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/requires: src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/TwistEndpoint.o.requires
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/requires

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/clean:
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints && $(CMAKE_COMMAND) -P CMakeFiles/diff_drive_message_endpoints.dir/cmake_clean.cmake
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/clean

src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/depend:
	cd /home/tobor/data-beagleboard/bb_navigation/diff_drive/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tobor/data-beagleboard/bb_navigation/diff_drive /home/tobor/data-beagleboard/bb_navigation/diff_drive/src/message_endpoints /home/tobor/data-beagleboard/bb_navigation/diff_drive/build /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints /home/tobor/data-beagleboard/bb_navigation/diff_drive/build/src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/message_endpoints/CMakeFiles/diff_drive_message_endpoints.dir/depend
