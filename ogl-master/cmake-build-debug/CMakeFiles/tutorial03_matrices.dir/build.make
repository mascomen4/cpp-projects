# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/ivan/ivan/CLion-2021.1.2/clion-2021.1.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/ivan/ivan/CLion-2021.1.2/clion-2021.1.2/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ivan/ivan/git/ogl-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ivan/ivan/git/ogl-master/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/tutorial03_matrices.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tutorial03_matrices.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tutorial03_matrices.dir/flags.make

CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.o: CMakeFiles/tutorial03_matrices.dir/flags.make
CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.o: ../tutorial03_matrices/tutorial03.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivan/ivan/git/ogl-master/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.o -c /home/ivan/ivan/git/ogl-master/tutorial03_matrices/tutorial03.cpp

CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivan/ivan/git/ogl-master/tutorial03_matrices/tutorial03.cpp > CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.i

CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivan/ivan/git/ogl-master/tutorial03_matrices/tutorial03.cpp -o CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.s

CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.o: CMakeFiles/tutorial03_matrices.dir/flags.make
CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.o: ../common/shader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivan/ivan/git/ogl-master/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.o -c /home/ivan/ivan/git/ogl-master/common/shader.cpp

CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivan/ivan/git/ogl-master/common/shader.cpp > CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.i

CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivan/ivan/git/ogl-master/common/shader.cpp -o CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.s

# Object files for target tutorial03_matrices
tutorial03_matrices_OBJECTS = \
"CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.o" \
"CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.o"

# External object files for target tutorial03_matrices
tutorial03_matrices_EXTERNAL_OBJECTS =

tutorial03_matrices: CMakeFiles/tutorial03_matrices.dir/tutorial03_matrices/tutorial03.cpp.o
tutorial03_matrices: CMakeFiles/tutorial03_matrices.dir/common/shader.cpp.o
tutorial03_matrices: CMakeFiles/tutorial03_matrices.dir/build.make
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libGL.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libGLU.so
tutorial03_matrices: external/glfw-3.1.2/src/libglfw3.a
tutorial03_matrices: external/libGLEW_1130.a
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/librt.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libm.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libX11.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libXrandr.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libXinerama.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libXi.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libXcursor.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/librt.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libm.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libX11.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libXrandr.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libXinerama.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libXi.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libXcursor.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libGL.so
tutorial03_matrices: /usr/lib/x86_64-linux-gnu/libGLU.so
tutorial03_matrices: CMakeFiles/tutorial03_matrices.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ivan/ivan/git/ogl-master/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable tutorial03_matrices"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tutorial03_matrices.dir/link.txt --verbose=$(VERBOSE)
	/home/ivan/ivan/CLion-2021.1.2/clion-2021.1.2/bin/cmake/linux/bin/cmake -E copy /home/ivan/ivan/git/ogl-master/cmake-build-debug/./tutorial03_matrices /home/ivan/ivan/git/ogl-master/tutorial03_matrices/

# Rule to build all files generated by this target.
CMakeFiles/tutorial03_matrices.dir/build: tutorial03_matrices

.PHONY : CMakeFiles/tutorial03_matrices.dir/build

CMakeFiles/tutorial03_matrices.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tutorial03_matrices.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tutorial03_matrices.dir/clean

CMakeFiles/tutorial03_matrices.dir/depend:
	cd /home/ivan/ivan/git/ogl-master/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ivan/ivan/git/ogl-master /home/ivan/ivan/git/ogl-master /home/ivan/ivan/git/ogl-master/cmake-build-debug /home/ivan/ivan/git/ogl-master/cmake-build-debug /home/ivan/ivan/git/ogl-master/cmake-build-debug/CMakeFiles/tutorial03_matrices.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tutorial03_matrices.dir/depend

