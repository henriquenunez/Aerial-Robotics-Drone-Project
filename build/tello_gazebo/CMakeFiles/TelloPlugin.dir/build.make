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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/src/tello_ros/tello_gazebo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/build/tello_gazebo

# Include any dependencies generated for this target.
include CMakeFiles/TelloPlugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TelloPlugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TelloPlugin.dir/flags.make

CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.o: CMakeFiles/TelloPlugin.dir/flags.make
CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.o: /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/src/tello_ros/tello_gazebo/src/tello_plugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/build/tello_gazebo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.o -c /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/src/tello_ros/tello_gazebo/src/tello_plugin.cpp

CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/src/tello_ros/tello_gazebo/src/tello_plugin.cpp > CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.i

CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/src/tello_ros/tello_gazebo/src/tello_plugin.cpp -o CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.s

# Object files for target TelloPlugin
TelloPlugin_OBJECTS = \
"CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.o"

# External object files for target TelloPlugin
TelloPlugin_EXTERNAL_OBJECTS =

libTelloPlugin.so: CMakeFiles/TelloPlugin.dir/src/tello_plugin.cpp.o
libTelloPlugin.so: CMakeFiles/TelloPlugin.dir/build.make
libTelloPlugin.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librclcpp.so
libTelloPlugin.so: /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/install/tello_msgs/lib/libtello_msgs__rosidl_typesupport_c.so
libTelloPlugin.so: /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/install/tello_msgs/lib/libtello_msgs__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgazebo_ros_node.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgazebo_ros_utils.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgazebo_ros_init.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgazebo_ros_factory.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgazebo_ros_properties.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgazebo_ros_state.so
libTelloPlugin.so: /opt/ros/galactic/lib/libgazebo_ros_force_system.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_generator_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcutils.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcpputils.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_runtime_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_generator_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl_yaml_param_parser.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_generator_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libtracetools.so
libTelloPlugin.so: /opt/ros/galactic/lib/librclcpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libament_index_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/liblibstatistics_collector.so
libTelloPlugin.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl_yaml_param_parser.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libtracetools.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libSimTKsimbody.so.3.6
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libdart.so.6.9.2
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_client.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gui.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_sensors.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_rendering.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_physics.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_ode.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_transport.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_msgs.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_util.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_common.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_gimpact.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opcode.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libgazebo_opende_ou.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.71.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libsdformat9.so.9.8.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libOgreMain.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libOgreTerrain.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libOgrePaging.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3-graphics.so.3.14.2
libTelloPlugin.so: /opt/ros/galactic/lib/libgeometry_msgs__rosidl_generator_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_generator_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librmw_implementation.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl_logging_spdlog.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcl_logging_interface.so
libTelloPlugin.so: /opt/ros/galactic/lib/libyaml.so
libTelloPlugin.so: /opt/ros/galactic/lib/librmw.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_generator_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_generator_c.so
libTelloPlugin.so: /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/install/tello_msgs/lib/libtello_msgs__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/install/tello_msgs/lib/libtello_msgs__rosidl_generator_c.so
libTelloPlugin.so: /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/install/tello_msgs/lib/libtello_msgs__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_generator_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_generator_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_typesupport_introspection_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_typesupport_introspection_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_typesupport_cpp.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_typesupport_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librosidl_runtime_c.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcpputils.so
libTelloPlugin.so: /opt/ros/galactic/lib/librcutils.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libSimTKmath.so.3.6
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libSimTKcommon.so.3.6
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libblas.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/liblapack.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libdart-external-odelcpsolver.so.6.9.2
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libccd.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libfcl.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libassimp.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.3
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/liboctomath.so.1.9.3
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-transport8.so.8.3.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-fuel_tools4.so.4.6.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-msgs5.so.5.10.0
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-math6.so.6.15.1
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libignition-common3.so.3.14.2
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libTelloPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libTelloPlugin.so: CMakeFiles/TelloPlugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/build/tello_gazebo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libTelloPlugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TelloPlugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TelloPlugin.dir/build: libTelloPlugin.so

.PHONY : CMakeFiles/TelloPlugin.dir/build

CMakeFiles/TelloPlugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TelloPlugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TelloPlugin.dir/clean

CMakeFiles/TelloPlugin.dir/depend:
	cd /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/build/tello_gazebo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/src/tello_ros/tello_gazebo /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/src/tello_ros/tello_gazebo /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/build/tello_gazebo /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/build/tello_gazebo /home/danial/drone_racing_ros2_ws/src/Aerial-Robotics-Drone-Project/build/tello_gazebo/CMakeFiles/TelloPlugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TelloPlugin.dir/depend

