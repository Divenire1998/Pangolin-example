############################################################
# 寻找库并包含路径
############################################################
#PANGOLIN
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# OPENCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(slamVisualization main.cpp slamVisualization.cpp)
target_link_libraries(slamVisualization ${Pangolin_LIBRARIES} ${OpenCV_LIBS})