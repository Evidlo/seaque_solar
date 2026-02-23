cmake_minimum_required(VERSION 3.12)

project(Drivers)
message("Reconfiguring device driver build system.")
enable_language(C)

# Glob sources from the HAL driver
file(GLOB hal_sources "STM32L5xx_HAL_Driver/Src/*.c")

add_library(Drivers OBJECT ${hal_sources})
target_compile_definitions(Drivers PUBLIC STM32L552xx USE_HAL_DRIVER USE_FULL_LL_DRIVER)

target_include_directories(Drivers
                            PUBLIC "CMSIS/Include"
                            PUBLIC "CMSIS/Device/ST/STM32L5xx/Include"
                            PUBLIC "STM32L5xx_HAL_Driver/Inc"
			    PUBLIC "../Core/Inc"
)
