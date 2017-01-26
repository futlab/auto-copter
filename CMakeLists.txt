project(auto-copter)

cmake_minimum_required(VERSION 2.8)

add_compile_options(-std=c++11)

find_package(uvc REQUIRED)
find_package(OpenCV REQUIRED)

aux_source_directory(. SRC_LIST)

#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${uvc_LIBRARIES} ${OpenCV_LIBS})
