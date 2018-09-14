cmake_minimum_required(VERSION 2.8)

project(slamevaluation)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
sensor_msgs
tf
roscpp
)

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(recordata predeal.cpp recordata.cpp main.cpp)

add_executable(registration evaluation.cpp registration.cpp predeal.cpp)

add_executable(evaluate evaluate.cpp evaluation.cpp predeal.cpp)

target_link_libraries(recordata ${PCL_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(registration ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#install(TARGETS slamevaluation RUNTIME DESTINATION bin)
