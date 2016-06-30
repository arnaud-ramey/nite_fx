PROJECT( nite_fx )
cmake_minimum_required(VERSION 2.6)
# Debug, Release, RelWithDebInfo and MinSizeRe
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings

FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(Boost REQUIRED COMPONENTS system)

INCLUDE_DIRECTORIES("/usr/include/ni")
INCLUDE_DIRECTORIES("/usr/include/nite")
INCLUDE_DIRECTORIES("/usr/include/openni")
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS})
CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/nite_fx_path.h.in"
               "${PROJECT_BINARY_DIR}/nite_fx_path.h")

ADD_SUBDIRECTORY(src)
