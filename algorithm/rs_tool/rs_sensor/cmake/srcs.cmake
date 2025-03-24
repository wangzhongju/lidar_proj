#========================
# controlcan libs
#========================
set(RS_DEP_LIBS "")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    list(APPEND RS_DEP_LIBS ${PROJECT_SOURCE_DIR}/lib/libcontrolcan_x86_64.so)
endif()
#========================
# libs
#========================

set(CUR_INCLUDES "")
set(CUR_SRCS "")
LIST(APPEND CUR_INCLUDES include)

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h ${dir}/*.hpp)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

add_library(${CUR_LIB} SHARED
        ${CUR_SRCS}
        )
target_include_directories(${CUR_LIB}
        PUBLIC
        ${CUR_INCLUDES}
        ${RS_DEP_INCLUDE_DIRS}
        )
target_link_libraries(${CUR_LIB}
        PUBLIC
        ${RS_DEP_LINKER_LIBS}
        ${RS_DEP_LIBS}
        rs_driver
        rs_common
        rs_dependence
        )
