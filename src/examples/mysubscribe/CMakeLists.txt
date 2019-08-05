px4_add_module(
	MODULE examples__mysub
	MAIN mysub
	STACK_MAIN 2000
	SRCS
		mysub_main.cpp
	DEPENDS
	)
