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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.16.5/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.16.5/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/dave/Animesh

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/dave/Animesh/cmake-build-debug

# Include any dependencies generated for this target.
include src/libCamera/CMakeFiles/Camera.dir/depend.make

# Include the progress variables for this target.
include src/libCamera/CMakeFiles/Camera.dir/progress.make

# Include the compile flags for this target's objects.
include src/libCamera/CMakeFiles/Camera.dir/flags.make

src/libCamera/CMakeFiles/Camera.dir/src/Camera.cpp.o: src/libCamera/CMakeFiles/Camera.dir/flags.make
src/libCamera/CMakeFiles/Camera.dir/src/Camera.cpp.o: ../src/libCamera/src/Camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/libCamera/CMakeFiles/Camera.dir/src/Camera.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/src/libCamera && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Camera.dir/src/Camera.cpp.o -c /Users/dave/Animesh/src/libCamera/src/Camera.cpp

src/libCamera/CMakeFiles/Camera.dir/src/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Camera.dir/src/Camera.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/src/libCamera && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/src/libCamera/src/Camera.cpp > CMakeFiles/Camera.dir/src/Camera.cpp.i

src/libCamera/CMakeFiles/Camera.dir/src/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Camera.dir/src/Camera.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/src/libCamera && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/src/libCamera/src/Camera.cpp -o CMakeFiles/Camera.dir/src/Camera.cpp.s

# Object files for target Camera
Camera_OBJECTS = \
"CMakeFiles/Camera.dir/src/Camera.cpp.o"

# External object files for target Camera
Camera_EXTERNAL_OBJECTS =

../lib/libCamera.a: src/libCamera/CMakeFiles/Camera.dir/src/Camera.cpp.o
../lib/libCamera.a: src/libCamera/CMakeFiles/Camera.dir/build.make
../lib/libCamera.a: src/libCamera/CMakeFiles/Camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../../lib/libCamera.a"
	cd /Users/dave/Animesh/cmake-build-debug/src/libCamera && $(CMAKE_COMMAND) -P CMakeFiles/Camera.dir/cmake_clean_target.cmake
	cd /Users/dave/Animesh/cmake-build-debug/src/libCamera && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/libCamera/CMakeFiles/Camera.dir/build: ../lib/libCamera.a

.PHONY : src/libCamera/CMakeFiles/Camera.dir/build

src/libCamera/CMakeFiles/Camera.dir/clean:
	cd /Users/dave/Animesh/cmake-build-debug/src/libCamera && $(CMAKE_COMMAND) -P CMakeFiles/Camera.dir/cmake_clean.cmake
.PHONY : src/libCamera/CMakeFiles/Camera.dir/clean

src/libCamera/CMakeFiles/Camera.dir/depend:
	cd /Users/dave/Animesh/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/dave/Animesh /Users/dave/Animesh/src/libCamera /Users/dave/Animesh/cmake-build-debug /Users/dave/Animesh/cmake-build-debug/src/libCamera /Users/dave/Animesh/cmake-build-debug/src/libCamera/CMakeFiles/Camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/libCamera/CMakeFiles/Camera.dir/depend

