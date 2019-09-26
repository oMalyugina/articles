cmake_minimum_required(VERSION 3.13)
project(egomotion)

find_package(HDF5 REQUIRED COMPONENTS CXX)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(roscpp)
find_package(catkin REQUIRED geometry_msgs tf_conversions tf tf2_msgs)
find_package(rosbag REQUIRED)

include_directories(
        include
        ${HDF5_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${rosbag_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS}
        include
        )


add_executable(hdfReader src/prepareDateForCartographer.cpp src/kittyToRosConverter.cpp)
#add_executable(hdfReader src/converterTFmessageToPose.cpp)

target_include_directories(hdfReader PUBLIC
        ${roscpp_INCLUDE_DIRS}
        include
        )

target_link_libraries(hdfReader
        ${HDF5_CXX_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${PCL_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${rosbag_LIBRARIES}
        ${catkin_LIBRARIES}
        )
