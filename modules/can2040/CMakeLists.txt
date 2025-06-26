if(CONFIG_CAN2040)
zephyr_library_named(can2040)

if(CONFIG_SOC_SERIES_RP2040)
zephyr_compile_definitions(PICO_RP2040)
elseif(CONFIG_SOC_SERIES_RP2350)
zephyr_compile_definitions(PICO_RP2350)
endif()

zephyr_include_directories(
	${ZEPHYR_CAN2040_MODULE_DIR}/src
)

zephyr_library_include_directories(
	${ZEPHYR_CAN2040_MODULE_DIR}/src
)

zephyr_library_sources(
	${ZEPHYR_CAN2040_MODULE_DIR}/src/can2040.c
)
endif()
