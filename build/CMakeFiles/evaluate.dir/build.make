# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/shaoan/projects/slamEvaluation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shaoan/projects/slamEvaluation/build

# Include any dependencies generated for this target.
include CMakeFiles/evaluate.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/evaluate.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/evaluate.dir/flags.make

CMakeFiles/evaluate.dir/evaluate.cpp.o: CMakeFiles/evaluate.dir/flags.make
CMakeFiles/evaluate.dir/evaluate.cpp.o: ../evaluate.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/shaoan/projects/slamEvaluation/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/evaluate.dir/evaluate.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/evaluate.dir/evaluate.cpp.o -c /home/shaoan/projects/slamEvaluation/evaluate.cpp

CMakeFiles/evaluate.dir/evaluate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/evaluate.dir/evaluate.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/shaoan/projects/slamEvaluation/evaluate.cpp > CMakeFiles/evaluate.dir/evaluate.cpp.i

CMakeFiles/evaluate.dir/evaluate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/evaluate.dir/evaluate.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/shaoan/projects/slamEvaluation/evaluate.cpp -o CMakeFiles/evaluate.dir/evaluate.cpp.s

CMakeFiles/evaluate.dir/evaluate.cpp.o.requires:
.PHONY : CMakeFiles/evaluate.dir/evaluate.cpp.o.requires

CMakeFiles/evaluate.dir/evaluate.cpp.o.provides: CMakeFiles/evaluate.dir/evaluate.cpp.o.requires
	$(MAKE) -f CMakeFiles/evaluate.dir/build.make CMakeFiles/evaluate.dir/evaluate.cpp.o.provides.build
.PHONY : CMakeFiles/evaluate.dir/evaluate.cpp.o.provides

CMakeFiles/evaluate.dir/evaluate.cpp.o.provides.build: CMakeFiles/evaluate.dir/evaluate.cpp.o

CMakeFiles/evaluate.dir/evaluation.cpp.o: CMakeFiles/evaluate.dir/flags.make
CMakeFiles/evaluate.dir/evaluation.cpp.o: ../evaluation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/shaoan/projects/slamEvaluation/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/evaluate.dir/evaluation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/evaluate.dir/evaluation.cpp.o -c /home/shaoan/projects/slamEvaluation/evaluation.cpp

CMakeFiles/evaluate.dir/evaluation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/evaluate.dir/evaluation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/shaoan/projects/slamEvaluation/evaluation.cpp > CMakeFiles/evaluate.dir/evaluation.cpp.i

CMakeFiles/evaluate.dir/evaluation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/evaluate.dir/evaluation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/shaoan/projects/slamEvaluation/evaluation.cpp -o CMakeFiles/evaluate.dir/evaluation.cpp.s

CMakeFiles/evaluate.dir/evaluation.cpp.o.requires:
.PHONY : CMakeFiles/evaluate.dir/evaluation.cpp.o.requires

CMakeFiles/evaluate.dir/evaluation.cpp.o.provides: CMakeFiles/evaluate.dir/evaluation.cpp.o.requires
	$(MAKE) -f CMakeFiles/evaluate.dir/build.make CMakeFiles/evaluate.dir/evaluation.cpp.o.provides.build
.PHONY : CMakeFiles/evaluate.dir/evaluation.cpp.o.provides

CMakeFiles/evaluate.dir/evaluation.cpp.o.provides.build: CMakeFiles/evaluate.dir/evaluation.cpp.o

CMakeFiles/evaluate.dir/predeal.cpp.o: CMakeFiles/evaluate.dir/flags.make
CMakeFiles/evaluate.dir/predeal.cpp.o: ../predeal.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/shaoan/projects/slamEvaluation/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/evaluate.dir/predeal.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/evaluate.dir/predeal.cpp.o -c /home/shaoan/projects/slamEvaluation/predeal.cpp

CMakeFiles/evaluate.dir/predeal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/evaluate.dir/predeal.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/shaoan/projects/slamEvaluation/predeal.cpp > CMakeFiles/evaluate.dir/predeal.cpp.i

CMakeFiles/evaluate.dir/predeal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/evaluate.dir/predeal.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/shaoan/projects/slamEvaluation/predeal.cpp -o CMakeFiles/evaluate.dir/predeal.cpp.s

CMakeFiles/evaluate.dir/predeal.cpp.o.requires:
.PHONY : CMakeFiles/evaluate.dir/predeal.cpp.o.requires

CMakeFiles/evaluate.dir/predeal.cpp.o.provides: CMakeFiles/evaluate.dir/predeal.cpp.o.requires
	$(MAKE) -f CMakeFiles/evaluate.dir/build.make CMakeFiles/evaluate.dir/predeal.cpp.o.provides.build
.PHONY : CMakeFiles/evaluate.dir/predeal.cpp.o.provides

CMakeFiles/evaluate.dir/predeal.cpp.o.provides.build: CMakeFiles/evaluate.dir/predeal.cpp.o

# Object files for target evaluate
evaluate_OBJECTS = \
"CMakeFiles/evaluate.dir/evaluate.cpp.o" \
"CMakeFiles/evaluate.dir/evaluation.cpp.o" \
"CMakeFiles/evaluate.dir/predeal.cpp.o"

# External object files for target evaluate
evaluate_EXTERNAL_OBJECTS =

evaluate: CMakeFiles/evaluate.dir/evaluate.cpp.o
evaluate: CMakeFiles/evaluate.dir/evaluation.cpp.o
evaluate: CMakeFiles/evaluate.dir/predeal.cpp.o
evaluate: CMakeFiles/evaluate.dir/build.make
evaluate: CMakeFiles/evaluate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable evaluate"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/evaluate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/evaluate.dir/build: evaluate
.PHONY : CMakeFiles/evaluate.dir/build

CMakeFiles/evaluate.dir/requires: CMakeFiles/evaluate.dir/evaluate.cpp.o.requires
CMakeFiles/evaluate.dir/requires: CMakeFiles/evaluate.dir/evaluation.cpp.o.requires
CMakeFiles/evaluate.dir/requires: CMakeFiles/evaluate.dir/predeal.cpp.o.requires
.PHONY : CMakeFiles/evaluate.dir/requires

CMakeFiles/evaluate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/evaluate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/evaluate.dir/clean

CMakeFiles/evaluate.dir/depend:
	cd /home/shaoan/projects/slamEvaluation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shaoan/projects/slamEvaluation /home/shaoan/projects/slamEvaluation /home/shaoan/projects/slamEvaluation/build /home/shaoan/projects/slamEvaluation/build /home/shaoan/projects/slamEvaluation/build/CMakeFiles/evaluate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/evaluate.dir/depend

