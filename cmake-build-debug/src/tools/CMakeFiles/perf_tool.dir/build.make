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
include src/tools/CMakeFiles/perf_tool.dir/depend.make

# Include the progress variables for this target.
include src/tools/CMakeFiles/perf_tool.dir/progress.make

# Include the compile flags for this target's objects.
include src/tools/CMakeFiles/perf_tool.dir/flags.make

src/tools/CMakeFiles/perf_tool.dir/perfTool.cpp.o: src/tools/CMakeFiles/perf_tool.dir/flags.make
src/tools/CMakeFiles/perf_tool.dir/perfTool.cpp.o: ../src/tools/perfTool.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/tools/CMakeFiles/perf_tool.dir/perfTool.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/src/tools && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/perf_tool.dir/perfTool.cpp.o -c /Users/dave/Animesh/src/tools/perfTool.cpp

src/tools/CMakeFiles/perf_tool.dir/perfTool.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/perf_tool.dir/perfTool.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/src/tools && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/src/tools/perfTool.cpp > CMakeFiles/perf_tool.dir/perfTool.cpp.i

src/tools/CMakeFiles/perf_tool.dir/perfTool.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/perf_tool.dir/perfTool.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/src/tools && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/src/tools/perfTool.cpp -o CMakeFiles/perf_tool.dir/perfTool.cpp.s

# Object files for target perf_tool
perf_tool_OBJECTS = \
"CMakeFiles/perf_tool.dir/perfTool.cpp.o"

# External object files for target perf_tool
perf_tool_EXTERNAL_OBJECTS =

../bin/perf_tool: src/tools/CMakeFiles/perf_tool.dir/perfTool.cpp.o
../bin/perf_tool: src/tools/CMakeFiles/perf_tool.dir/build.make
../bin/perf_tool: ../lib/libArgs.a
../bin/perf_tool: ../lib/libField.a
../bin/perf_tool: ../lib/libFileUtils.a
../bin/perf_tool: ../lib/libGraph.a
../bin/perf_tool: ../lib/libRoSy.a
../bin/perf_tool: ../lib/libField.a
../bin/perf_tool: ../lib/libRoSy.a
../bin/perf_tool: ../lib/libFileUtils.a
../bin/perf_tool: ../lib/libGraph.a
../bin/perf_tool: ../lib/libGeom.a
../bin/perf_tool: src/tools/CMakeFiles/perf_tool.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/perf_tool"
	cd /Users/dave/Animesh/cmake-build-debug/src/tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/perf_tool.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/tools/CMakeFiles/perf_tool.dir/build: ../bin/perf_tool

.PHONY : src/tools/CMakeFiles/perf_tool.dir/build

src/tools/CMakeFiles/perf_tool.dir/clean:
	cd /Users/dave/Animesh/cmake-build-debug/src/tools && $(CMAKE_COMMAND) -P CMakeFiles/perf_tool.dir/cmake_clean.cmake
.PHONY : src/tools/CMakeFiles/perf_tool.dir/clean

src/tools/CMakeFiles/perf_tool.dir/depend:
	cd /Users/dave/Animesh/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/dave/Animesh /Users/dave/Animesh/src/tools /Users/dave/Animesh/cmake-build-debug /Users/dave/Animesh/cmake-build-debug/src/tools /Users/dave/Animesh/cmake-build-debug/src/tools/CMakeFiles/perf_tool.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/tools/CMakeFiles/perf_tool.dir/depend

