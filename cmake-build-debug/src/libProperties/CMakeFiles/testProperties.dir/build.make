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
include src/libProperties/CMakeFiles/testProperties.dir/depend.make

# Include the progress variables for this target.
include src/libProperties/CMakeFiles/testProperties.dir/progress.make

# Include the compile flags for this target's objects.
include src/libProperties/CMakeFiles/testProperties.dir/flags.make

src/libProperties/CMakeFiles/testProperties.dir/tests/TestProperties.cpp.o: src/libProperties/CMakeFiles/testProperties.dir/flags.make
src/libProperties/CMakeFiles/testProperties.dir/tests/TestProperties.cpp.o: ../src/libProperties/tests/TestProperties.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/libProperties/CMakeFiles/testProperties.dir/tests/TestProperties.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testProperties.dir/tests/TestProperties.cpp.o -c /Users/dave/Animesh/src/libProperties/tests/TestProperties.cpp

src/libProperties/CMakeFiles/testProperties.dir/tests/TestProperties.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testProperties.dir/tests/TestProperties.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/src/libProperties/tests/TestProperties.cpp > CMakeFiles/testProperties.dir/tests/TestProperties.cpp.i

src/libProperties/CMakeFiles/testProperties.dir/tests/TestProperties.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testProperties.dir/tests/TestProperties.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/src/libProperties/tests/TestProperties.cpp -o CMakeFiles/testProperties.dir/tests/TestProperties.cpp.s

src/libProperties/CMakeFiles/testProperties.dir/tests/main.cpp.o: src/libProperties/CMakeFiles/testProperties.dir/flags.make
src/libProperties/CMakeFiles/testProperties.dir/tests/main.cpp.o: ../src/libProperties/tests/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/libProperties/CMakeFiles/testProperties.dir/tests/main.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testProperties.dir/tests/main.cpp.o -c /Users/dave/Animesh/src/libProperties/tests/main.cpp

src/libProperties/CMakeFiles/testProperties.dir/tests/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testProperties.dir/tests/main.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/src/libProperties/tests/main.cpp > CMakeFiles/testProperties.dir/tests/main.cpp.i

src/libProperties/CMakeFiles/testProperties.dir/tests/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testProperties.dir/tests/main.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/src/libProperties/tests/main.cpp -o CMakeFiles/testProperties.dir/tests/main.cpp.s

# Object files for target testProperties
testProperties_OBJECTS = \
"CMakeFiles/testProperties.dir/tests/TestProperties.cpp.o" \
"CMakeFiles/testProperties.dir/tests/main.cpp.o"

# External object files for target testProperties
testProperties_EXTERNAL_OBJECTS =

../bin/testProperties: src/libProperties/CMakeFiles/testProperties.dir/tests/TestProperties.cpp.o
../bin/testProperties: src/libProperties/CMakeFiles/testProperties.dir/tests/main.cpp.o
../bin/testProperties: src/libProperties/CMakeFiles/testProperties.dir/build.make
../bin/testProperties: ../lib/libProperties.a
../bin/testProperties: src/libProperties/CMakeFiles/testProperties.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../../../bin/testProperties"
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testProperties.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Copying properties unit test data."
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && /usr/local/Cellar/cmake/3.16.5/bin/cmake -E copy_directory /Users/dave/Animesh/src/libProperties/tests/test_data /Users/dave/Animesh/cmake-build-debug/properties_test_data

# Rule to build all files generated by this target.
src/libProperties/CMakeFiles/testProperties.dir/build: ../bin/testProperties

.PHONY : src/libProperties/CMakeFiles/testProperties.dir/build

src/libProperties/CMakeFiles/testProperties.dir/clean:
	cd /Users/dave/Animesh/cmake-build-debug/src/libProperties && $(CMAKE_COMMAND) -P CMakeFiles/testProperties.dir/cmake_clean.cmake
.PHONY : src/libProperties/CMakeFiles/testProperties.dir/clean

src/libProperties/CMakeFiles/testProperties.dir/depend:
	cd /Users/dave/Animesh/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/dave/Animesh /Users/dave/Animesh/src/libProperties /Users/dave/Animesh/cmake-build-debug /Users/dave/Animesh/cmake-build-debug/src/libProperties /Users/dave/Animesh/cmake-build-debug/src/libProperties/CMakeFiles/testProperties.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/libProperties/CMakeFiles/testProperties.dir/depend

