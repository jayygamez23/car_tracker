project (program3)
cmake_minimum_required(VERSION 3.15)

# configure OpenCV
find_package(OpenCV REQUIRED)

# create create individual projects
add_executable(program3 program3.cpp)
target_link_libraries(program3 ${OpenCV_LIBS})


