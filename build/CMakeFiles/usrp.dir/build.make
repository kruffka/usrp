# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/novosibirsk/rx_tx_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/novosibirsk/rx_tx_test/build

# Include any dependencies generated for this target.
include CMakeFiles/usrp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/usrp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/usrp.dir/flags.make

CMakeFiles/usrp.dir/usrp.cpp.o: CMakeFiles/usrp.dir/flags.make
CMakeFiles/usrp.dir/usrp.cpp.o: ../usrp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/novosibirsk/rx_tx_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/usrp.dir/usrp.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/usrp.dir/usrp.cpp.o -c /home/novosibirsk/rx_tx_test/usrp.cpp

CMakeFiles/usrp.dir/usrp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/usrp.dir/usrp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/novosibirsk/rx_tx_test/usrp.cpp > CMakeFiles/usrp.dir/usrp.cpp.i

CMakeFiles/usrp.dir/usrp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/usrp.dir/usrp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/novosibirsk/rx_tx_test/usrp.cpp -o CMakeFiles/usrp.dir/usrp.cpp.s

CMakeFiles/usrp.dir/usrp.cpp.o.requires:

.PHONY : CMakeFiles/usrp.dir/usrp.cpp.o.requires

CMakeFiles/usrp.dir/usrp.cpp.o.provides: CMakeFiles/usrp.dir/usrp.cpp.o.requires
	$(MAKE) -f CMakeFiles/usrp.dir/build.make CMakeFiles/usrp.dir/usrp.cpp.o.provides.build
.PHONY : CMakeFiles/usrp.dir/usrp.cpp.o.provides

CMakeFiles/usrp.dir/usrp.cpp.o.provides.build: CMakeFiles/usrp.dir/usrp.cpp.o


# Object files for target usrp
usrp_OBJECTS = \
"CMakeFiles/usrp.dir/usrp.cpp.o"

# External object files for target usrp
usrp_EXTERNAL_OBJECTS =

usrp: CMakeFiles/usrp.dir/usrp.cpp.o
usrp: CMakeFiles/usrp.dir/build.make
usrp: /usr/local/lib/libuhd.so
usrp: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
usrp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
usrp: /usr/lib/x86_64-linux-gnu/libboost_system.so
usrp: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
usrp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
usrp: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
usrp: /usr/lib/x86_64-linux-gnu/libpthread.so
usrp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
usrp: CMakeFiles/usrp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/novosibirsk/rx_tx_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable usrp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/usrp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/usrp.dir/build: usrp

.PHONY : CMakeFiles/usrp.dir/build

CMakeFiles/usrp.dir/requires: CMakeFiles/usrp.dir/usrp.cpp.o.requires

.PHONY : CMakeFiles/usrp.dir/requires

CMakeFiles/usrp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/usrp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/usrp.dir/clean

CMakeFiles/usrp.dir/depend:
	cd /home/novosibirsk/rx_tx_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/novosibirsk/rx_tx_test /home/novosibirsk/rx_tx_test /home/novosibirsk/rx_tx_test/build /home/novosibirsk/rx_tx_test/build /home/novosibirsk/rx_tx_test/build/CMakeFiles/usrp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/usrp.dir/depend

