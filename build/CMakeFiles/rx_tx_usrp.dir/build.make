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
include CMakeFiles/rx_tx_usrp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rx_tx_usrp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rx_tx_usrp.dir/flags.make

CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o: CMakeFiles/rx_tx_usrp.dir/flags.make
CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o: ../txrx_loopback_to_file.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/novosibirsk/rx_tx_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o -c /home/novosibirsk/rx_tx_test/txrx_loopback_to_file.cpp

CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/novosibirsk/rx_tx_test/txrx_loopback_to_file.cpp > CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.i

CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/novosibirsk/rx_tx_test/txrx_loopback_to_file.cpp -o CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.s

CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o.requires:

.PHONY : CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o.requires

CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o.provides: CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o.requires
	$(MAKE) -f CMakeFiles/rx_tx_usrp.dir/build.make CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o.provides.build
.PHONY : CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o.provides

CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o.provides.build: CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o


# Object files for target rx_tx_usrp
rx_tx_usrp_OBJECTS = \
"CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o"

# External object files for target rx_tx_usrp
rx_tx_usrp_EXTERNAL_OBJECTS =

rx_tx_usrp: CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o
rx_tx_usrp: CMakeFiles/rx_tx_usrp.dir/build.make
rx_tx_usrp: /usr/local/lib/libuhd.so
rx_tx_usrp: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
rx_tx_usrp: /usr/lib/x86_64-linux-gnu/libboost_thread.so
rx_tx_usrp: /usr/lib/x86_64-linux-gnu/libboost_system.so
rx_tx_usrp: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
rx_tx_usrp: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
rx_tx_usrp: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
rx_tx_usrp: /usr/lib/x86_64-linux-gnu/libpthread.so
rx_tx_usrp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
rx_tx_usrp: CMakeFiles/rx_tx_usrp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/novosibirsk/rx_tx_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rx_tx_usrp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rx_tx_usrp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rx_tx_usrp.dir/build: rx_tx_usrp

.PHONY : CMakeFiles/rx_tx_usrp.dir/build

CMakeFiles/rx_tx_usrp.dir/requires: CMakeFiles/rx_tx_usrp.dir/txrx_loopback_to_file.cpp.o.requires

.PHONY : CMakeFiles/rx_tx_usrp.dir/requires

CMakeFiles/rx_tx_usrp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rx_tx_usrp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rx_tx_usrp.dir/clean

CMakeFiles/rx_tx_usrp.dir/depend:
	cd /home/novosibirsk/rx_tx_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/novosibirsk/rx_tx_test /home/novosibirsk/rx_tx_test /home/novosibirsk/rx_tx_test/build /home/novosibirsk/rx_tx_test/build /home/novosibirsk/rx_tx_test/build/CMakeFiles/rx_tx_usrp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rx_tx_usrp.dir/depend

