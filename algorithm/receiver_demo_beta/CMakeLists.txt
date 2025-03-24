set(ALL_LIBS
        rs_application
        ${RS_DEP_LINKER_LIBS}
        )
set(ALL_INCLUDES
        ${RS_DEP_INCLUDE_DIRS}
        )

add_executable(receiver_demo receiver_test_demo.cpp)
target_include_directories(receiver_demo PRIVATE ${ALL_INCLUDES})
target_link_libraries(receiver_demo PRIVATE ${ALL_LIBS} pthread)
