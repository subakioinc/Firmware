px4_add_module(
	MODULE examples__simple_subscribe
	MAIN simple_subscribe
	STACK_MAIN 2000
	SRCS
		simple_subscribe.cpp
	DEPENDS
	)
