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
include third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/depend.make

# Include the progress variables for this target.
include third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/progress.make

# Include the compile flags for this target's objects.
include third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/main.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/main.cpp.o: ../third_party/nanogui/python/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/main.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/main.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/main.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/main.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/main.cpp > CMakeFiles/nanogui-python-obj.dir/python/main.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/main.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/main.cpp -o CMakeFiles/nanogui-python-obj.dir/python/main.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.o: ../third_party/nanogui/python/constants_glfw.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/constants_glfw.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/constants_glfw.cpp > CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/constants_glfw.cpp -o CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.o: ../third_party/nanogui/python/constants_entypo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/constants_entypo.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/constants_entypo.cpp > CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/constants_entypo.cpp -o CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.o: ../third_party/nanogui/python/eigen.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/eigen.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/eigen.cpp > CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/eigen.cpp -o CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.o: ../third_party/nanogui/python/widget.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/widget.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/widget.cpp > CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/widget.cpp -o CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.o: ../third_party/nanogui/python/layout.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/layout.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/layout.cpp > CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/layout.cpp -o CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.o: ../third_party/nanogui/python/basics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/basics.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/basics.cpp > CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/basics.cpp -o CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/button.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/button.cpp.o: ../third_party/nanogui/python/button.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/button.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/button.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/button.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/button.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/button.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/button.cpp > CMakeFiles/nanogui-python-obj.dir/python/button.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/button.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/button.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/button.cpp -o CMakeFiles/nanogui-python-obj.dir/python/button.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.o: ../third_party/nanogui/python/tabs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/tabs.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/tabs.cpp > CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/tabs.cpp -o CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.o: ../third_party/nanogui/python/textbox.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/textbox.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/textbox.cpp > CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/textbox.cpp -o CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.o: ../third_party/nanogui/python/theme.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/theme.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/theme.cpp > CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/theme.cpp -o CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.o: ../third_party/nanogui/python/glcanvas.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/glcanvas.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/glcanvas.cpp > CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/glcanvas.cpp -o CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.o: ../third_party/nanogui/python/formhelper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/formhelper.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/formhelper.cpp > CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/formhelper.cpp -o CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.o: ../third_party/nanogui/python/misc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/misc.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/misc.cpp > CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/misc.cpp -o CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.o: ../third_party/nanogui/python/glutil.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/glutil.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/glutil.cpp > CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/glutil.cpp -o CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.o: ../third_party/nanogui/python/nanovg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.o -c /Users/dave/Animesh/third_party/nanogui/python/nanovg.cpp

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/python/nanovg.cpp > CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/python/nanovg.cpp -o CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.s

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.o: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/flags.make
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.o: ../third_party/nanogui/ext/coro/coro.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building C object third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.o   -c /Users/dave/Animesh/third_party/nanogui/ext/coro/coro.c

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/coro/coro.c > CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.i

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/coro/coro.c -o CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.s

nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/main.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_glfw.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/constants_entypo.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/eigen.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/widget.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/layout.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/basics.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/button.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/tabs.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/textbox.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/theme.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glcanvas.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/formhelper.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/misc.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/glutil.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/python/nanovg.cpp.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/ext/coro/coro.c.o
nanogui-python-obj: third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/build.make

.PHONY : nanogui-python-obj

# Rule to build all files generated by this target.
third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/build: nanogui-python-obj

.PHONY : third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/build

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/clean:
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui && $(CMAKE_COMMAND) -P CMakeFiles/nanogui-python-obj.dir/cmake_clean.cmake
.PHONY : third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/clean

third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/depend:
	cd /Users/dave/Animesh/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/dave/Animesh /Users/dave/Animesh/third_party/nanogui /Users/dave/Animesh/cmake-build-debug /Users/dave/Animesh/cmake-build-debug/third_party/nanogui /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : third_party/nanogui/CMakeFiles/nanogui-python-obj.dir/depend
