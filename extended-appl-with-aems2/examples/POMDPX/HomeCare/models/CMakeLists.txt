cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions( -std=c++11 )
endif()


project( DisplayImage )
find_package( OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

add_executable(simulate simulateMovments.cpp)
add_executable(makeModel_A6 simpleHomeSw.cpp) # only 6 actions are in use
add_executable(makeModel_A7 HomeSw_NESWFXO.cpp) # all 7 actions are in use

target_link_libraries( simulate opencv_core opencv_highgui opencv_imgproc)
#target_link_libraries( makeModel tinyxml)
