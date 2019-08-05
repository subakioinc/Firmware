px4_add_module(
	MODULE examples__mypub
	MAIN mypub
	STACK_MAIN 2000
	SRCS
		mypub_main.cpp
	DEPENDS
)
