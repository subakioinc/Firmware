px4_add_module(
    MODULE examples__simple_cpp_app
    MAIN simple_cpp_app
    STACK_MAIN 2000
    SRCS
        simple_cpp_app.cpp
    )
