#
# Copyright (c) 2021 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0
#

add_library(tfm_app_rot_partition_dp STATIC)

target_include_directories(tfm_app_rot_partition_dp
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    PRIVATE
        ${CMAKE_BINARY_DIR}/dummy_partition
)
target_include_directories(tfm_partitions
    INTERFACE
        ${CMAKE_BINARY_DIR}/dummy_partition
)

# # Map Zephyr targets to LLVM targets.
# if(CONFIG_CPU_CORTEX_M)
#   if(CONFIG_CPU_CORTEX_M23)
#     set(RUST_TARGET "thumbv8m.base-none-eabi" PARENT_SCOPE)
#   elseif(CONFIG_CPU_CORTEX_M33 OR CONFIG_CPU_CORTEX_M55)
#     # Not a typo, Zephyr, uses ARMV7_M_ARMV8_M_FP to select the FP even on v8m.
#     if(CONFIG_FP_HARDABI OR FORCE_FP_HARDABI)
#       set(RUST_TARGET "thumbv8m.main-none-eabihf" PARENT_SCOPE)
#     else()
      set(RUST_TARGET "thumbv8m.main-none-eabi")# PARENT_SCOPE)
#     endif()

#     # Todo: The M55 is thumbv8.1m.main-none-eabi, which can be added when Rust
#     # gain support for this target.
#   else()
#     message(FATAL_ERROR "Unknown Cortex-M target.")
#   endif()
# else()
#   message(FATAL_ERROR "Rust TZ partitions only supported on Cortex-M")
# endif()

if(CONFIG_DEBUG)
  set(RUST_BUILD_TYPE "debug")
  set(rust_build_type_arg "")
else()
  set(RUST_BUILD_TYPE "release")
  set(rust_build_type_arg "--release")
endif()

set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/rust/target")
set(RUST_LIBRARY "${CARGO_TARGET_DIR}/${RUST_TARGET}/${RUST_BUILD_TYPE}/librustlet.a")
set(DUMMY_FILE "${CMAKE_BINARY_DIR}/always-run-cargo.dummy")

set(RUST_CFLAGS -fshort-enums)
list(APPEND RUST_CFLAGS -DMBEDTLS_CONFIG_FILE=\\\"/tmp/zephyr/zephyr/modules/mbedtls/configs/config-mbedtls.h\\\"${llext_filt_flags})

add_custom_command(
  OUTPUT ${DUMMY_FILE}
  BYPRODUCTS ${RUST_LIBRARY} ${WRAPPER_FILE}
  USES_TERMINAL
  COMMAND
    ${CMAKE_COMMAND} -E
    env BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}
    ZEPHYR_BASE=${ZEPHYR_BASE}
    DOTCONFIG=${DOTCONFIG}
    ZEPHYR_DTS=${ZEPHYR_DTS}
    INCLUDE_DIRS="${include_dirs}"
    INCLUDE_DEFINES="${include_defines}"
    WRAPPER_FILE="${WRAPPER_FILE}"
    DT_AUGMENTS="${DT_AUGMENTS}"
    BINARY_DIR_INCLUDE_GENERATED="${BINARY_DIR_INCLUDE_GENERATED}"
    BINDGEN_EXTRA_CLANG_ARGS="-DMBEDTLS_CONFIG_FILE=</tmp/zephyr/zephyr/modules/mbedtls/configs/config-mbedtls.h>"
    CC=clang
    CFLAGS="${RUST_CFLAGS}"
    MBEDTLS_LIB_DIR=/tmp/tmpdir
    MBEDTLS_INCLUDE_DIR=/tmp/zephyr/modules/tee/tf-m/trusted-firmware-m/interface/include
    cargo build
    ${rust_build_type_arg}

    # Override the features according to the shield given. For a general case,
    # this will need to come from a variable or argument.
    # TODO: This needs to be passed in.
    # --no-default-features
    # --features ${SHIELD_FEATURE}

    # Set a replacement so that packages can just use `zephyr-sys` as a package
    # name to find it.
    ${command_paths}
    --target ${RUST_TARGET}
    --target-dir ${CARGO_TARGET_DIR}
  COMMENT "Building Rust application"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# Be sure we don't try building this until all of the generated headers have been generated.
add_custom_target(librustlet ALL
  DEPENDS ${DUMMY_FILE}
)

add_dependencies(tfm_app_rot_partition_dp librustlet)

# add_library(librustlet STATIC IMPORTED GLOBAL)
# set_target_properties(librustlet PROPERTIES
#   IMPORTED_LOCATION /tmp/zephyr/zephyr/samples/tfm_integration/tfm_secure_partition/dummy_partition/target/thumbv8m.main-none-eabi/debug/librustlet.a
# )

target_link_libraries(tfm_app_rot_partition_dp
    PRIVATE
        ${RUST_LIBRARY}
)

# The generated sources
target_sources(tfm_app_rot_partition_dp
    PRIVATE
        ${CMAKE_BINARY_DIR}/dummy_partition/auto_generated/intermedia_tfm_dummy_partition.c
)
target_sources(tfm_partitions
    INTERFACE
        ${CMAKE_BINARY_DIR}/dummy_partition/auto_generated/load_info_tfm_dummy_partition.c
)

target_link_libraries(tfm_app_rot_partition_dp
    PRIVATE
        platform_s
        tfm_sprt
        psa_crypto_config
)

target_link_libraries(tfm_spm
    PRIVATE
        tfm_app_rot_partition_dp
)

############################ Partition Defs ####################################

target_link_libraries(tfm_partitions
    INTERFACE
        tfm_app_rot_partition_dp
)

target_compile_definitions(tfm_config
    INTERFACE
        TFM_PARTITION_DUMMY_PARTITION
)
