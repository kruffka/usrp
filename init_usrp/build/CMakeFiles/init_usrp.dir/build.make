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
CMAKE_SOURCE_DIR = /home/novosibirsk/rx_tx_test/init_usrp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/novosibirsk/rx_tx_test/init_usrp/build

# Include any dependencies generated for this target.
include CMakeFiles/init_usrp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/init_usrp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/init_usrp.dir/flags.make

CMakeFiles/init_usrp.dir/init_usrp.cpp.o: CMakeFiles/init_usrp.dir/flags.make
CMakeFiles/init_usrp.dir/init_usrp.cpp.o: ../init_usrp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/novosibirsk/rx_tx_test/init_usrp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/init_usrp.dir/init_usrp.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/init_usrp.dir/init_usrp.cpp.o -c /home/novosibirsk/rx_tx_test/init_usrp/init_usrp.cpp

CMakeFiles/init_usrp.dir/init_usrp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/init_usrp.dir/init_usrp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/novosibirsk/rx_tx_test/init_usrp/init_usrp.cpp > CMakeFiles/init_usrp.dir/init_usrp.cpp.i

CMakeFiles/init_usrp.dir/init_usrp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/init_usrp.dir/init_usrp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/novosibirsk/rx_tx_test/init_usrp/init_usrp.cpp -o CMakeFiles/init_usrp.dir/init_usrp.cpp.s

CMakeFiles/init_usrp.dir/init_usrp.cpp.o.requires:

.PHONY : CMakeFiles/init_usrp.dir/init_usrp.cpp.o.requires

CMakeFiles/init_usrp.dir/init_usrp.cpp.o.provides: CMakeFiles/init_usrp.dir/init_usrp.cpp.o.requires
	$(MAKE) -f CMakeFiles/init_usrp.dir/build.make CMakeFiles/init_usrp.dir/init_usrp.cpp.o.provides.build
.PHONY : CMakeFiles/init_usrp.dir/init_usrp.cpp.o.provides

CMakeFiles/init_usrp.dir/init_usrp.cpp.o.provides.build: CMakeFiles/init_usrp.dir/init_usrp.cpp.o


# Object files for target init_usrp
init_usrp_OBJECTS = \
"CMakeFiles/init_usrp.dir/init_usrp.cpp.o"

# External object files for target init_usrp
init_usrp_EXTERNAL_OBJECTS =

init_usrp: CMakeFiles/init_usrp.dir/init_usrp.cpp.o
init_usrp: CMakeFiles/init_usrp.dir/build.make
init_usrp: /usr/local/lib/libuhd.so
init_usrp: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
init_usrp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
init_usrp: /usr/lib/x86_64-linux-gnu/libboost_system.so
init_usrp: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
init_usrp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
init_usrp: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
init_usrp: /usr/lib/x86_64-linux-gnu/libpthread.so
init_usrp: CMakeFiles/init_usrp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/novosibirsk/rx_tx_test/init_usrp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable init_usrp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/init_usrp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/init_usrp.dir/build: init_usrp

.PHONY : CMakeFiles/init_usrp.dir/build

CMakeFiles/init_usrp.dir/requires: CMakeFiles/init_usrp.dir/init_usrp.cpp.o.requires

.PHONY : CMakeFiles/init_usrp.dir/requires

CMakeFiles/init_usrp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/init_usrp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/init_usrp.dir/clean

CMakeFiles/init_usrp.dir/depend:
	cd /home/novosibirsk/rx_tx_test/init_usrp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/novosibirsk/rx_tx_test/init_usrp /home/novosibirsk/rx_tx_test/init_usrp /home/novosibirsk/rx_tx_test/init_usrp/build /home/novosibirsk/rx_tx_test/init_usrp/build /home/novosibirsk/rx_tx_test/init_usrp/build/CMakeFiles/init_usrp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/init_usrp.dir/depend

