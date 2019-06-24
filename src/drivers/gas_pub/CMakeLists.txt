px4_add_module(
    MODULE drivers__gas_pub
    MAIN gas_pub
    STACK_MAIN 2000
    SRCS
    		gas_pub_main.cpp
    DEPENDS

)
