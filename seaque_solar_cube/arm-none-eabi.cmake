MESSAGE("[!] READING THE CROSS-COMPILATION TOOLCHAIN")

SET(CMAKE_SYSTEM_PROCESSOR  arm)
SET(CMAKE_SYSTEM_NAME       Generic)

SET(CMAKE_TRY_COMPILE_TARGET_TYPE   STATIC_LIBRARY)

SET(ARCH        armv8-m.main)
SET(CPU         cortex-m33)
SET(ARM_ISA     mthumb)
SET(FPU         fpv5-sp-d16)
SET(FLOAT_ABI   hard)
SET(SPECS       nano.specs)

SET(CMAKE_C_COMPILER    arm-none-eabi-gcc)
SET(CMAKE_CXX_COMPILER  arm-none-eabi-g++)
SET(CMAKE_ASM_COMPILER  arm-none-eabi-g++)
SET(CMAKE_SIZE          arm-none-eabi-size)
SET(CMAKE_OBJDUMP       arm-none-eabi-objdump)
SET(CMAKE_OBJCOPY       arm-none-eabi-objcopy)

SET(OPTIMISATION Og)
SET(DEBUG "ggdb")

SET(CMAKE_COMMON_FLAGS "-mcpu=${CPU} -${ARM_ISA} -${OPTIMISATION} -${DEBUG} -Wall -Wextra -ffunction-sections -fdata-sections --specs=${SPECS} -mfpu=${FPU} -mfloat-abi=${FLOAT_ABI} -u _printf_float -u _scanf_float -lc -lm")
SET(CMAKE_ASM_FLAGS     "${CMAKE_COMMON_FLAGS}")
SET(CMAKE_C_FLAGS       "${CMAKE_COMMON_FLAGS}") #-std=gnull
add_link_options(LINKER:--print-memory-usage LINKER:-Map,${TARGET}.map, )
