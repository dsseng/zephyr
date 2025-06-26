if(CONFIG_CAN_CAN2040)
zephyr_library_named(can2040)


zephyr_include_directories(
	${ZEPHYR_CAN2040_MODULE_DIR}/src
)

zephyr_library_sources(
	${ZEPHYR_CAN2040_MODULE_DIR}/src/can2040.c
)
endif()
