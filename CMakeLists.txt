cmake_minimum_required(VERSION 2.8)
project(pangolin_use)


############################################################
# Build Type
############################################################
IF(NOT CMAKE_BUILD_TYPE)
SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
############################################################
#  C/C++ Flags
############################################################
# -O3 编译等级优化为3上 开启了这个就不能用debug,会丢失信息了
# -Wall 开启所有警告
# -march=native, which automatically detects the features your CPU supports and sets the options appropriately.
if(CMAKE_BUILD_TYPE MATCHES Release)
   MESSAGE("开启编译器优化")
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
endif()
########################################s####################
# Check C++11 or C++0x support
############################################################
include(CheckCXXCompilerFlag)
   CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
   CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
   if(COMPILER_SUPPORTS_CXX11)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
      add_definitions(-DCOMPILEDWITHC11)
      message(STATUS "Using flag -std=c++11.")
   elseif(COMPILER_SUPPORTS_CXX0X)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
      add_definitions(-DCOMPILEDWITHC0X)
      message(STATUS "Using flag -std=c++0x.")
   else()
      message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

############################################################
# Cmake Moudule Path
############################################################
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
message(${CMAKE_MODULE_PATH})

############################################################
# 寻找库并包含路径
############################################################
#PANGOLIN
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

############################################################
# 输出可执行文件
############################################################
## 二进制文件输出路径
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/exe)
add_executable(demo01_HelloPangolin demo01_HelloPangolin.cpp)
target_link_libraries(demo01_HelloPangolin ${Pangolin_LIBRARIES})

add_executable(demo02_HelloPangolinThreads demo02_HelloPangolinThreads.cpp)
target_link_libraries(demo02_HelloPangolinThreads ${Pangolin_LIBRARIES})

add_executable(demo03_SimpleGuiUse demo03_SimpleGuiUse.cpp)
target_link_libraries(demo03_SimpleGuiUse ${Pangolin_LIBRARIES})

