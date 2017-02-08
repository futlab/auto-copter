project(auto-copter)

cmake_minimum_required(VERSION 2.8)

#find_package(uvc REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


add_subdirectory(modules/apriltags/src)


add_compile_options(-std=c++11)


aux_source_directory(. SRC_LIST)


#aux_source_directory(modules/witrack WITRACK_SRC_LIST)
set(SRC_LIST ${SRC_LIST}
    modules/witrack/witrack/frame.cpp
    modules/witrack/witrack/conveyor.cpp
    modules/witrack/witrack/jacobi.cpp
    modules/witrack/witrack/visualizer.cpp
)
include_directories(modules/witrack/witrack ${Eigen3_INCLUDE_DIRS})



#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${uvc_LIBRARIES} apriltags ${OpenCV_LIBS})
