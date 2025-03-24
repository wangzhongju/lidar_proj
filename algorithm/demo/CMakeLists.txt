set(ALL_LIBS
        rs_application
        ${RS_DEP_LINKER_LIBS}
        )
set(ALL_INCLUDES
        ${RS_DEP_INCLUDE_DIRS}
        )

add_executable(rs_sdk_demo rs_sdk_demo.cpp)
target_include_directories(rs_sdk_demo PRIVATE ${ALL_INCLUDES})
target_link_libraries(rs_sdk_demo PRIVATE ${ALL_LIBS} pthread)

#add_executable(auto_align auto_align.cpp)
#target_include_directories(auto_align PRIVATE ${ALL_INCLUDES})
#target_link_libraries(auto_align PRIVATE ${ALL_LIBS} pthread)

