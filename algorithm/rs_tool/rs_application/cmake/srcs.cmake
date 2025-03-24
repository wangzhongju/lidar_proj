#========================
# libs
#========================

set(CUR_SRCS "")
set(CUR_INCLUDES "include")

set(CUR_SUB_DIR "")
LIST(APPEND CUR_SUB_DIR include)
LIST(APPEND CUR_SUB_DIR src)

foreach (dir ${CUR_SUB_DIR})
    file(GLOB_RECURSE tmp_srcs ${dir}/*.cpp ${dir}/*.h)
    list(APPEND CUR_SRCS ${tmp_srcs})
endforeach ()

add_library(${CUR_LIB} SHARED
        ${CUR_SRCS}
        )
target_include_directories(${CUR_LIB}
        PUBLIC
        ${CUR_INCLUDES}
        ${RS_DEP_INCLUDE_DIRS}
        ${release_sdk_config_dir}
        )
target_link_libraries(${CUR_LIB}
        PUBLIC
        ${RS_DEP_LINKER_LIBS}
        rs_common
        rs_preprocessing
        rs_perception_common
        rs_perception_lidar_ai_detection
        rs_perception_lidar_beta
        rs_perception_lidar_perception
        rs_beta
        rs_sensor
        rviz_display
        rs_custom
        rs_dependence
        )
if (COMPILE_LOCALIZATION)
    target_link_libraries(${CUR_LIB}
            PUBLIC
            rs_localization
            )
endif ()