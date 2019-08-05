px4_add_module(
    MODULE examples__simple_c_app
    MAIN simple_c_app
    STACK_MAIN 2000
    SRCS
        simple_c_app.c
    )
