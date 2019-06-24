px4_add_module(
	MODULE drivers__gas_sub
	MAIN gas_sub
	STACK_MAIN 2000
	SRCS
			gas_sub_main.cpp
	DEPENDS
)
