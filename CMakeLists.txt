cmake_minimum_required(VERSION 2.8)
project(object_detection)

## 添加c++ 11标准支持
#ADD_COMPILE_OPTIONS(-std=c++11 )
#ADD_COMPILE_OPTIONS(-std=c++14 )
#add_definitions(-std=c++03)
#set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set( CMAKE_CXX_FLAGS "-std=c++11" )
#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)
# 设置编译器编译模式
set( CMAKE_BUILD_TYPE "Debug" )
# 设置动态链接库输出目录
SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED PATHS /usr/local/opencv-3.4.5)
# 寻找PCL
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ELAS
set (LIBELAS_DIR "third_party/libelas")

# 添加头文件
include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${LIBELAS_DIR}/include
)

aux_source_directory(${LIBELAS_DIR}/src LIBELAS_SRC_DIR)
add_library(ELAS_LIBS SHARED ${LIBELAS_SRC_DIR})

aux_source_directory(${PROJECT_SOURCE_DIR}/src MYLIB_SRC_DIR)
add_library(MY_LIBS SHARED ${MYLIB_SRC_DIR})
target_link_libraries(MY_LIBS
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        ELAS_LIBS
        )

# stereo_calib_with_opencv
add_executable(stereo_calib
        examples/stereo_calib.cpp
        )

target_link_libraries(
        stereo_calib
        ${OpenCV_LIBS}
)

# camera R t calibration wrt. word coordinates
add_executable(Rt_calib
        examples/Rt_calib.cpp
        )

target_link_libraries(
        Rt_calib
        MY_LIBS
)

# gen_sparities
add_executable(gen_sparities
        examples/gen_sparities.cpp
)

target_link_libraries(
        gen_sparities
        MY_LIBS
)

# object_detection
add_executable(object_detection
        examples/object_detection.cpp
)

target_link_libraries(
        object_detection
        MY_LIBS
)
