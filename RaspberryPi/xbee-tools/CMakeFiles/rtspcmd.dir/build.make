# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools

# Include any dependencies generated for this target.
include CMakeFiles/rtspcmd.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rtspcmd.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rtspcmd.dir/flags.make

CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o: CMakeFiles/rtspcmd.dir/flags.make
CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o: xbeemux-core/Utils.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o -c /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeemux-core/Utils.cpp

CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeemux-core/Utils.cpp > CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.i

CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeemux-core/Utils.cpp -o CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.s

CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o.requires:
.PHONY : CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o.requires

CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o.provides: CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtspcmd.dir/build.make CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o.provides.build
.PHONY : CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o.provides

CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o.provides.build: CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o

CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o: CMakeFiles/rtspcmd.dir/flags.make
CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o: xbeertsp/rtspcmd.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o -c /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/rtspcmd.cpp

CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/rtspcmd.cpp > CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.i

CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/rtspcmd.cpp -o CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.s

CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o.requires:
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o.requires

CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o.provides: CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtspcmd.dir/build.make CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o.provides.build
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o.provides

CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o.provides.build: CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o

CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o: CMakeFiles/rtspcmd.dir/flags.make
CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o: xbeertsp/DeviceInfo.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o -c /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/DeviceInfo.cpp

CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/DeviceInfo.cpp > CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.i

CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/DeviceInfo.cpp -o CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.s

CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o.requires:
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o.requires

CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o.provides: CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtspcmd.dir/build.make CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o.provides.build
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o.provides

CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o.provides.build: CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o

CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o: CMakeFiles/rtspcmd.dir/flags.make
CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o: xbeertsp/HexImage.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o -c /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/HexImage.cpp

CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/HexImage.cpp > CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.i

CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/HexImage.cpp -o CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.s

CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o.requires:
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o.requires

CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o.provides: CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtspcmd.dir/build.make CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o.provides.build
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o.provides

CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o.provides.build: CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o

CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o: CMakeFiles/rtspcmd.dir/flags.make
CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o: xbeertsp/NotifyStatus.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o -c /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/NotifyStatus.cpp

CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/NotifyStatus.cpp > CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.i

CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/NotifyStatus.cpp -o CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.s

CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o.requires:
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o.requires

CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o.provides: CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtspcmd.dir/build.make CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o.provides.build
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o.provides

CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o.provides.build: CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o

CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o: CMakeFiles/rtspcmd.dir/flags.make
CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o: xbeertsp/Utils.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o -c /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/Utils.cpp

CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/Utils.cpp > CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.i

CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/xbeertsp/Utils.cpp -o CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.s

CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o.requires:
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o.requires

CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o.provides: CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o.requires
	$(MAKE) -f CMakeFiles/rtspcmd.dir/build.make CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o.provides.build
.PHONY : CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o.provides

CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o.provides.build: CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o

# Object files for target rtspcmd
rtspcmd_OBJECTS = \
"CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o" \
"CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o" \
"CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o" \
"CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o" \
"CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o" \
"CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o"

# External object files for target rtspcmd
rtspcmd_EXTERNAL_OBJECTS =

rtspcmd: CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o
rtspcmd: CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o
rtspcmd: CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o
rtspcmd: CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o
rtspcmd: CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o
rtspcmd: CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o
rtspcmd: CMakeFiles/rtspcmd.dir/build.make
rtspcmd: /usr/lib/x86_64-linux-gnu/liblzo2.so
rtspcmd: CMakeFiles/rtspcmd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable rtspcmd"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rtspcmd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rtspcmd.dir/build: rtspcmd
.PHONY : CMakeFiles/rtspcmd.dir/build

CMakeFiles/rtspcmd.dir/requires: CMakeFiles/rtspcmd.dir/xbeemux-core/Utils.cpp.o.requires
CMakeFiles/rtspcmd.dir/requires: CMakeFiles/rtspcmd.dir/xbeertsp/rtspcmd.cpp.o.requires
CMakeFiles/rtspcmd.dir/requires: CMakeFiles/rtspcmd.dir/xbeertsp/DeviceInfo.cpp.o.requires
CMakeFiles/rtspcmd.dir/requires: CMakeFiles/rtspcmd.dir/xbeertsp/HexImage.cpp.o.requires
CMakeFiles/rtspcmd.dir/requires: CMakeFiles/rtspcmd.dir/xbeertsp/NotifyStatus.cpp.o.requires
CMakeFiles/rtspcmd.dir/requires: CMakeFiles/rtspcmd.dir/xbeertsp/Utils.cpp.o.requires
.PHONY : CMakeFiles/rtspcmd.dir/requires

CMakeFiles/rtspcmd.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rtspcmd.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rtspcmd.dir/clean

CMakeFiles/rtspcmd.dir/depend:
	cd /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools /home/miguelprieto/eurobot/firmware/2017/RaspberryPi/xbee-tools/CMakeFiles/rtspcmd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rtspcmd.dir/depend

