cmake_minimum_required(VERSION 3.20)

project(STM32_implementation LANGUAGES C ASM)

if(NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR
        "Use the ARM GCC toolchain: -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake")
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build configuration" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HAL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver/Src")
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32L476RGTX_FLASH.ld")

add_executable(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES
    SUFFIX ".elf"
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS YES
)

# Compile each implementation separately; main.c includes their headers.
target_sources(${PROJECT_NAME} PRIVATE
    App/lidar.c
    BSP/bsp_lidar.c
    Core/Src/main.c
    Core/Src/stm32l4xx_hal_msp.c
    Core/Src/stm32l4xx_it.c
    Core/Src/syscalls.c
    Core/Src/sysmem.c
    Core/Src/system_stm32l4xx.c
    Core/Startup/startup_stm32l476rgtx.s

    # Match the HAL modules selected by the existing CubeIDE build.
    ${HAL_SOURCE_DIR}/stm32l4xx_hal.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_cortex.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_dma.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_dma_ex.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_exti.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_flash.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_flash_ex.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_flash_ramfunc.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_gpio.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_i2c.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_i2c_ex.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_pwr.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_pwr_ex.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_rcc.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_rcc_ex.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_tim.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_tim_ex.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_uart.c
    ${HAL_SOURCE_DIR}/stm32l4xx_hal_uart_ex.c
)

# These paths let main.c and the other modules find App and BSP headers.
target_include_directories(${PROJECT_NAME} PRIVATE
    App
    BSP
    Core/Inc
    Drivers/STM32L4xx_HAL_Driver/Inc
    Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32L4xx/Include
    Drivers/CMSIS/Include
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32L476xx
    $<$<CONFIG:Debug>:DEBUG>
)

# Match the STM32L476 Cortex-M4 and hardware floating-point ABI in CubeIDE.
set(MCU_OPTIONS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${MCU_OPTIONS}
    $<$<COMPILE_LANGUAGE:C>:-Wall>
    $<$<COMPILE_LANGUAGE:C>:-ffunction-sections>
    $<$<COMPILE_LANGUAGE:C>:-fdata-sections>
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Debug>:-g3>
    --specs=nano.specs
)

# The linker script places code, RAM data, and interrupt vectors on the MCU.
target_link_options(${PROJECT_NAME} PRIVATE
    ${MCU_OPTIONS}
    "-T${LINKER_SCRIPT}"
    --specs=nano.specs
    --specs=nosys.specs
    -static
    -Wl,--gc-sections
    "-Wl,-Map=$<TARGET_FILE_DIR:${PROJECT_NAME}>/${PROJECT_NAME}.map"
    -Wl,--print-memory-usage
)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS "${LINKER_SCRIPT}")
target_link_libraries(${PROJECT_NAME} PRIVATE m)
