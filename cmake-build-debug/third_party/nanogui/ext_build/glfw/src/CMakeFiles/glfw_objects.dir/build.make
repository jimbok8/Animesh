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
include third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/depend.make

# Include the progress variables for this target.
include third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/progress.make

# Include the compile flags for this target's objects.
include third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o: ../third_party/nanogui/ext/glfw/src/context.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/context.c.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/context.c

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/context.c.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/context.c > CMakeFiles/glfw_objects.dir/context.c.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/context.c.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/context.c -o CMakeFiles/glfw_objects.dir/context.c.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o: ../third_party/nanogui/ext/glfw/src/init.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/init.c.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/init.c

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/init.c.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/init.c > CMakeFiles/glfw_objects.dir/init.c.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/init.c.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/init.c -o CMakeFiles/glfw_objects.dir/init.c.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o: ../third_party/nanogui/ext/glfw/src/input.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/input.c.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/input.c

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/input.c.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/input.c > CMakeFiles/glfw_objects.dir/input.c.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/input.c.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/input.c -o CMakeFiles/glfw_objects.dir/input.c.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o: ../third_party/nanogui/ext/glfw/src/monitor.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/monitor.c.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/monitor.c

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/monitor.c.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/monitor.c > CMakeFiles/glfw_objects.dir/monitor.c.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/monitor.c.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/monitor.c -o CMakeFiles/glfw_objects.dir/monitor.c.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o: ../third_party/nanogui/ext/glfw/src/vulkan.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/vulkan.c.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/vulkan.c

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/vulkan.c.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/vulkan.c > CMakeFiles/glfw_objects.dir/vulkan.c.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/vulkan.c.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/vulkan.c -o CMakeFiles/glfw_objects.dir/vulkan.c.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o: ../third_party/nanogui/ext/glfw/src/window.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/window.c.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/window.c

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/window.c.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/window.c > CMakeFiles/glfw_objects.dir/window.c.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/window.c.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/window.c -o CMakeFiles/glfw_objects.dir/window.c.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_init.m.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_init.m.o: ../third_party/nanogui/ext/glfw/src/cocoa_init.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_init.m.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/cocoa_init.m.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_init.m

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_init.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/cocoa_init.m.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_init.m > CMakeFiles/glfw_objects.dir/cocoa_init.m.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_init.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/cocoa_init.m.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_init.m -o CMakeFiles/glfw_objects.dir/cocoa_init.m.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_joystick.m.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_joystick.m.o: ../third_party/nanogui/ext/glfw/src/cocoa_joystick.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_joystick.m.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/cocoa_joystick.m.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_joystick.m

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_joystick.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/cocoa_joystick.m.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_joystick.m > CMakeFiles/glfw_objects.dir/cocoa_joystick.m.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_joystick.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/cocoa_joystick.m.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_joystick.m -o CMakeFiles/glfw_objects.dir/cocoa_joystick.m.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_monitor.m.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_monitor.m.o: ../third_party/nanogui/ext/glfw/src/cocoa_monitor.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_monitor.m.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/cocoa_monitor.m.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_monitor.m

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_monitor.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/cocoa_monitor.m.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_monitor.m > CMakeFiles/glfw_objects.dir/cocoa_monitor.m.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_monitor.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/cocoa_monitor.m.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_monitor.m -o CMakeFiles/glfw_objects.dir/cocoa_monitor.m.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_window.m.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_window.m.o: ../third_party/nanogui/ext/glfw/src/cocoa_window.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_window.m.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/cocoa_window.m.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_window.m

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_window.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/cocoa_window.m.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_window.m > CMakeFiles/glfw_objects.dir/cocoa_window.m.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_window.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/cocoa_window.m.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_window.m -o CMakeFiles/glfw_objects.dir/cocoa_window.m.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_time.c.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_time.c.o: ../third_party/nanogui/ext/glfw/src/cocoa_time.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_time.c.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/cocoa_time.c.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_time.c

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_time.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/cocoa_time.c.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_time.c > CMakeFiles/glfw_objects.dir/cocoa_time.c.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_time.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/cocoa_time.c.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/cocoa_time.c -o CMakeFiles/glfw_objects.dir/cocoa_time.c.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o: ../third_party/nanogui/ext/glfw/src/posix_tls.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/posix_tls.c.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/posix_tls.c

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/posix_tls.c.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/posix_tls.c > CMakeFiles/glfw_objects.dir/posix_tls.c.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/posix_tls.c.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/posix_tls.c -o CMakeFiles/glfw_objects.dir/posix_tls.c.s

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/nsgl_context.m.o: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/flags.make
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/nsgl_context.m.o: ../third_party/nanogui/ext/glfw/src/nsgl_context.m
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/dave/Animesh/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building C object third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/nsgl_context.m.o"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/glfw_objects.dir/nsgl_context.m.o   -c /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/nsgl_context.m

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/nsgl_context.m.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/glfw_objects.dir/nsgl_context.m.i"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/nsgl_context.m > CMakeFiles/glfw_objects.dir/nsgl_context.m.i

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/nsgl_context.m.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/glfw_objects.dir/nsgl_context.m.s"
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/dave/Animesh/third_party/nanogui/ext/glfw/src/nsgl_context.m -o CMakeFiles/glfw_objects.dir/nsgl_context.m.s

glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/context.c.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/init.c.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/input.c.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/monitor.c.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/vulkan.c.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/window.c.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_init.m.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_joystick.m.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_monitor.m.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_window.m.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/cocoa_time.c.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/posix_tls.c.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/nsgl_context.m.o
glfw_objects: third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/build.make

.PHONY : glfw_objects

# Rule to build all files generated by this target.
third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/build: glfw_objects

.PHONY : third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/build

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/clean:
	cd /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/glfw_objects.dir/cmake_clean.cmake
.PHONY : third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/clean

third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/depend:
	cd /Users/dave/Animesh/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/dave/Animesh /Users/dave/Animesh/third_party/nanogui/ext/glfw/src /Users/dave/Animesh/cmake-build-debug /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src /Users/dave/Animesh/cmake-build-debug/third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : third_party/nanogui/ext_build/glfw/src/CMakeFiles/glfw_objects.dir/depend

