#===================================
#  perception lib path
#===================================

set(PERCEPTION_LIBS_PATH ${PROJECT_SOURCE_DIR}/libs/perception)
set(PERCEPTION_INCLUDE_PATH ${PERCEPTION_LIBS_PATH}/include)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(PERCEPTION_LIBS_PATH ${PERCEPTION_LIBS_PATH}/lib/x86_64)
endif ()
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(PERCEPTION_LIBS_PATH ${PERCEPTION_LIBS_PATH}/lib/aarch64)
endif ()



#========================
# add rs_common rs_driver rs_preprocessing
#========================
if (NOT INTERFACE_FOUND)
    ## rs_common
    add_library(rs_common SHARED IMPORTED GLOBAL)
    set_target_properties(rs_common PROPERTIES
            IMPORTED_LOCATION ${PERCEPTION_LIBS_PATH}/librs_common.so)
    set_target_properties(rs_common PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${PERCEPTION_INCLUDE_PATH})
    ## rs_driver
    add_library(rs_driver SHARED IMPORTED GLOBAL)
    set_target_properties(rs_driver PROPERTIES
            IMPORTED_LOCATION ${PERCEPTION_LIBS_PATH}/librs_driver.so)
    set_target_properties(rs_driver PROPERTIES
            INTERFACE_LINK_LIBRARIES "rs_common"
            )
    ## rs_preprocessing
    add_library(rs_preprocessing SHARED IMPORTED GLOBAL)
    set_target_properties(rs_preprocessing PROPERTIES
            IMPORTED_LOCATION ${PERCEPTION_LIBS_PATH}/librs_preprocessing.so)
    set_target_properties(rs_preprocessing PROPERTIES
            INTERFACE_LINK_LIBRARIES "rs_common"
            )
endif ()

#========================
# add rs perception
#========================
if (NOT PERCEPTION_INTERFACE_FOUND)
    ## rs perception common
    add_library(rs_perception_common SHARED IMPORTED GLOBAL)
    set_target_properties(rs_perception_common PROPERTIES
            IMPORTED_LOCATION ${PERCEPTION_LIBS_PATH}/librs_perception_common.so)
    set_target_properties(rs_perception_common PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${PERCEPTION_INCLUDE_PATH})
    set_target_properties(rs_perception_common PROPERTIES
            INTERFACE_LINK_LIBRARIES "rs_common"
            )

    ## rs perception infer
    add_library(rs_perception_infer SHARED IMPORTED GLOBAL)
    set_target_properties(rs_perception_infer PROPERTIES
            IMPORTED_LOCATION ${PERCEPTION_LIBS_PATH}/librs_perception_infer.so)
    set_target_properties(rs_perception_infer PROPERTIES
            INTERFACE_LINK_LIBRARIES "rs_common;rs_perception_common"
            )

    ## rs perception ai detection
    add_library(rs_perception_lidar_ai_detection SHARED IMPORTED GLOBAL)
    set_target_properties(rs_perception_lidar_ai_detection PROPERTIES
            IMPORTED_LOCATION ${PERCEPTION_LIBS_PATH}/librs_perception_lidar_ai_detection.so)
    set_target_properties(rs_perception_lidar_ai_detection PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${PERCEPTION_INCLUDE_PATH})
    set_target_properties(rs_perception_lidar_ai_detection PROPERTIES
            INTERFACE_LINK_LIBRARIES "rs_common;rs_perception_common;rs_perception_infer"
            )
    ## rs_perception beta
    add_library(rs_perception_lidar_beta SHARED IMPORTED GLOBAL)
    set_target_properties(rs_perception_lidar_beta PROPERTIES
            IMPORTED_LOCATION ${PERCEPTION_LIBS_PATH}/librs_perception_lidar_beta.so)
    set_target_properties(rs_perception_lidar_beta PROPERTIES
            INTERFACE_LINK_LIBRARIES "rs_common;rs_perception_common"
            )

    ## rs_perception lidar perception
    add_library(rs_perception_lidar_perception SHARED IMPORTED GLOBAL)
    set_target_properties(rs_perception_lidar_perception PROPERTIES
            IMPORTED_LOCATION ${PERCEPTION_LIBS_PATH}/librs_perception_lidar_perception.so)
    set_target_properties(rs_perception_lidar_perception PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${PERCEPTION_INCLUDE_PATH})
    set_target_properties(rs_perception_lidar_perception PROPERTIES
            INTERFACE_LINK_LIBRARIES "rs_common;rs_perception_common"
            )
endif ()

#===================================
#  localization lib path
#===================================
set(LOCALIZATION_LIB_PATH ${PROJECT_SOURCE_DIR}/libs/localization/lib)
set(LOCALIZATION_INCLUDE_PATH ${PROJECT_SOURCE_DIR}/libs/localization/include)

#========================
# add rs localization
#========================
if (NOT LOCALIZATION_INTERFACE_FOUND)
    #    execute_process(COMMAND ln -s ${PROJECT_SOURCE_DIR}/libs/localization/localization_config
    #            ${PROJECT_SOURCE_DIR}/config/system_config/localization_config)
    add_library(rs_localization SHARED IMPORTED GLOBAL)
    find_program(LSB_EXEC lsb_release)
    if(LSB_EXEC MATCHES "NOTFOUND")
        message("\n lsb_release not found, please install using: \n\t sudo apt install lsb_release\n")
    endif()
    execute_process(
            COMMAND ${LSB_EXEC} -cs
            OUTPUT_VARIABLE LSB_CODENAME
            OUTPUT_STRIP_TRAILING_WHITESPACE)
    set_target_properties(rs_localization PROPERTIES
            IMPORTED_LOCATION ${LOCALIZATION_LIB_PATH}/${CMAKE_SYSTEM_PROCESSOR}/${LSB_CODENAME}/librs_localization.so)
    set_target_properties(rs_localization PROPERTIES
            IMPORTED_LOCATION ${LOCALIZATION_LIB_PATH}/${CMAKE_SYSTEM_PROCESSOR}/${LSB_CODENAME}/librs_localization.so)
    set_target_properties(rs_localization PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${LOCALIZATION_INCLUDE_PATH})
endif ()


