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
include src/libProperties/CMakeFiles/Properties.dir/depend.make

# Include the progress variables for this target.
include src/libProperties/CMakeFiles/Properties.dir/progress.make

# Include the compile flags for this target's objects.
include src/libProperties/CMakeFiles/Properties.dir/flags.make

src/libProperties/CMakeFiles/Properties.dir/src/Properties.cpp.o: src/libProperties/CMakeFiles/Properties.dir/flags.make
src/libProperties/CMakeFiles/Properties.dir/src/Properties.cpp.o: ../src/libProperties/src/Properties.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/libProperties/CMakeFiles/Properties.dir/src/Properties.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Properties.dir/src/Properties.cpp.o -c /Users/dave/Animesh/src/libProperties/src/Properties.cpp

src/libProperties/CMakeFiles/Properties.dir/src/Properties.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Properties.dir/src/Properties.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/src/libProperties/src/Properties.cpp > CMakeFiles/Properties.dir/src/Properties.cpp.i

src/libProperties/CMakeFiles/Properties.dir/src/Properties.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Properties.dir/src/Properties.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/src/libProperties/src/Properties.cpp -o CMakeFiles/Properties.dir/src/Properties.cpp.s

# Object files for target Properties
Properties_OBJECTS = \
"CMakeFiles/Properties.dir/src/Properties.cpp.o"

# External object files for target Properties
Properties_EXTERNAL_OBJECTS =

../lib/libProperties.a: src/libProperties/CMakeFiles/Properties.dir/src/Properties.cpp.o
../lib/libProperties.a: src/libProperties/CMakeFiles/Properties.dir/build.make
../lib/libProperties.a: src/libProperties/CMakeFiles/Properties.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../../lib/libProperties.a"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && $(CMAKE_COMMAND) -P CMakeFiles/Properties.dir/cmake_clean_target.cmake
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Properties.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/libProperties/CMakeFiles/Properties.dir/build: ../lib/libProperties.a

.PHONY : src/libProperties/CMakeFiles/Properties.dir/build

src/libProperties/CMakeFiles/Properties.dir/clean:
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && $(CMAKE_COMMAND) -P CMakeFiles/Properties.dir/cmake_clean.cmake
.PHONY : src/libProperties/CMakeFiles/Properties.dir/clean

src/libProperties/CMakeFiles/Properties.dir/depend:
	cd /Users/dave/Animesh/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/dave/Animesh /Users/dave/Animesh/src/libProperties /Users/dave/Animesh/cmake-build-debug /Users/dave/Animesh/cmake-build-debug/src/libProperties /Users/dave/Animesh/cmake-build-debug/src/libProperties/CMakeFiles/Properties.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/libProperties/CMakeFiles/Properties.dir/depend

