set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)

set(CMAKE_EXE_LINKER_FLAGS
    "-specs=nosys.specs -specs=nano.specs -mcpu=cortex-m4 -mfloat-abi=hard -mthumb -mfpu=fpv4-sp-d16"
    CACHE INTERNAL "")

add_compile_options(
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
    -specs=nosys.specs	
    -ffunction-sections
    -Wall
    -Wextra
    -Wno-unused-parameter
	-fmessage-length=0
    )

add_compile_definitions(
    "__weak=__attribute__((weak))"
    "__packed=__attribute__((__packed__))"
    USE_HAL_DRIVER
    STM32F407xx
    )
