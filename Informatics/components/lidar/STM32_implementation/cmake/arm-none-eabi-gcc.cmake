# Cross-compile for a bare-metal ARM target, rather than the host PC.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Compiler checks must not try to link or execute a host-style program.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc REQUIRED)
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}")
