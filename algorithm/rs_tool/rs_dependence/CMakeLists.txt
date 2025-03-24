cmake_minimum_required(VERSION 3.5)

#========================
# Project
#========================

set(CUR_LIB rs_dependence)
project(${CUR_LIB})

set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++14)
add_compile_options(-W)
add_compile_options(-std=c++14)
add_definitions(-DDEBUG_LEVEL=0)

set(CMAKE_INSTALL_PREFIX ${RS_CMAKE_INSTALL_PREFIX})

include(cmake/setting.cmake)
include(cmake/dependence.cmake)
include(cmake/srcs.cmake)

#========================
# configure
#========================
configure_file (
        "${PROJECT_SOURCE_DIR}/include/rs_dependence/utils.h.in"
        "${PROJECT_SOURCE_DIR}/include/rs_dependence/utils.h"
)

set(${CUR_LIB} PARENT_SCOPE)
set(RS_DEP_INCLUDE_DIRS ${RS_DEP_INCLUDE_DIRS} PARENT_SCOPE)
set(RS_DEP_LINKER_LIBS ${RS_DEP_LINKER_LIBS} PARENT_SCOPE)


