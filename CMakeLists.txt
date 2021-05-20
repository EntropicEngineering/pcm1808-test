# Required by cmake
cmake_minimum_required(VERSION 3.13.1)

# Hardware definition
set(BOARD nrf52840dk_nrf52840)
set(CONFIG_UART_CONSOLE n CACHE STRING "Force no UART console")

# Define Zephyr's location in the repo.
get_filename_component(ZEPHYR_BASE ${CMAKE_SOURCE_DIR}/../zephyr ABSOLUTE CACHE)
# Find Zephyr. This also loads Zephyr's build system.
find_package(Zephyr REQUIRED NO_DEFAULT_PATH PATHS ${ZEPHYR_BASE})

project(pcm1808-test)

# Add your source file to the "app" target. This must come after
# find_package(Zephyr) which defines the target.
target_sources(app PRIVATE
    src/main.c
    )
