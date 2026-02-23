cmake_minimum_required(VERSION 3.13)
# Stop CMake from trying to test the compiler
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

enable_language(C ASM)

# Main flight software build toolchain
# Hopefully this is good enough to tide us over

project(qcb VERSION 1.0.0 DESCRIPTION "Flight Software for STM32L552 QCB")
message("Reconfiguring top-level flight software build process")
set(TARGET qcb)

# Use the toolchain
set(CMAKE_TOOLCHAIN_FILE "${PROJECT_SOURCE_DIR}/arm-none-eabi.cmake")

# Select which linker script to use
option(RAM_ONLY "Do not store program to flash, load directly to RAM" OFF)
if (${RAM_ONLY})
    set(LINKER_SCRIPT "${PROJECT_SOURCE_DIR}/STM32L552ZETXQ_RAM.ld")
else()
    set(LINKER_SCRIPT "${PROJECT_SOURCE_DIR}/STM32L552ZETXQ_FLASH.ld")
endif()


# Add subdirectories
add_subdirectory(Drivers)
add_subdirectory(libs/seaque_icomms/lib)
add_subdirectory(libs/axe/lib)
file(GLOB CoreSources CONFIGURE_DEPENDS "Core/Src/*.c")
file(GLOB StartupSources CONFIGURE_DEPENDS "Core/Startup/*.s")
list(APPEND SRCS ${CoreSources} ${StartupSources} "Core/Src/axe_init.c")

# May be needed to snprintf floats
#add_link_options("-u _printf_float")
list(APPEND STMHDRS "Drivers/CMSIS/Device/ST/STM32L5xx/Include" "Core/Inc" "Drivers/STM32L5xx_HAL_Driver/Inc" "Drivers/CMSIS/Include")
target_compile_definitions(seaque-icomm PUBLIC STM32_BOARD STM32L552xx USE_HAL_DRIVER USE_FULL_LL_DRIVER N_INTERFACES=2)
add_executable(${TARGET}.elf ${SRCS})

# Add object library dependencies from the other directories
list(APPEND OBJECT_LIBS Drivers)
target_link_libraries(${TARGET}.elf PUBLIC ${OBJECT_LIBS} seaque-icomm axe)
target_include_directories(${TARGET}.elf PUBLIC "${PROJECT_SOURCE_DIR}/Core/Inc")
target_include_directories(seaque-icomm PUBLIC ${STMHDRS} )
target_compile_definitions(${TARGET}.elf PUBLIC STM32L552xx USE_HAL_DRIVER USE_FULL_LL_DRIVER)

# Add linker script
set_target_properties(${TARGET}.elf PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})
target_link_options(${TARGET}.elf PRIVATE 
                    -T ${LINKER_SCRIPT})

# This should create bin and hex files
ADD_CUSTOM_COMMAND(OUTPUT ${TARGET}.bin DEPENDS ${TARGET}.elf COMMAND ${CMAKE_OBJCOPY} -O binary ${TARGET}.elf ${TARGET}.bin)
ADD_CUSTOM_COMMAND(OUTPUT ${TARGET}.hex DEPENDS ${TARGET}.elf COMMAND ${CMAKE_OBJCOPY} -O ihex ${TARGET}.elf ${TARGET}.hex)
ADD_CUSTOM_TARGET(${TARGET}_bin DEPENDS ${TARGET}.bin)
ADD_CUSTOM_TARGET(${TARGET}_hex DEPENDS ${TARGET}.hex)

add_custom_command(
OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/axe_init.c ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/axe_init.h
COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/libs/axe/table_gen.py ${CMAKE_CURRENT_SOURCE_DIR}/table.tsv ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc/axe_init.h ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/axe_init.c
DEPENDS table.tsv
  VERBATIM)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()
