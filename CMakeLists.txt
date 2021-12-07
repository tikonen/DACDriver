# Configure and build (for Ninja in git-bash)
#
# $ export PATH=/c/opt/GNU\ MCU\ Eclipse/ARM\ Embedded\ GCC/8.2.1-1.2-20190119-1237/bin/:$PATH
# $ mkdir build
# $ cd build
# $ cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain-gnu-stm.cmake -G Ninja ..
#-- The C compiler identification is GNU 8.2.1
#-- The ASM compiler identification is GNU
#-- Found assembler: C:/opt/GNU MCU Eclipse/ARM Embedded GCC/8.2.1-1.2-20190119-1237/bin/arm-none-eabi-gcc.exe
#-- Detecting C compiler ABI info
#-- Detecting C compiler ABI info - done
#-- Check for working C compiler: C:/opt/GNU MCU Eclipse/ARM Embedded GCC/8.2.1-1.2-20190119-1237/bin/arm-none-eabi-gcc.exe - skipped
#-- Detecting C compile features
#-- Detecting C compile features - done
#-- Build type: Release
#-- Configuring done
#-- Generating done
#-- Build files have been written to: C:/DACDriver/build
# $ ninja
# [1/37] Building C object CMakeFiles/DACDriver.elf.dir/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c.obj
# [2/37] Building C object CMakeFiles/DACDriver.elf.dir/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c.obj
# [3/37] Building C object CMakeFiles/DACDriver.elf.dir/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c.obj
# [4/37] Building C object CMakeFiles/DACDriver.elf.dir/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c.obj
# ...


cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

project(DACDriver LANGUAGES C ASM)

set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

add_executable(${PROJECT_NAME}.elf)

target_compile_options(${PROJECT_NAME}.elf PUBLIC $<$<CONFIG:Debug>:-Og -gdwarf-4 -g3 -fvar-tracking-assignments>)
target_compile_options(${PROJECT_NAME}.elf PUBLIC $<$<CONFIG:Release>:-O>)

file(
    GLOB_RECURSE _source_list
    LIST_DIRECTORIES false
	"${CMAKE_SOURCE_DIR}/Drivers/*.c"
	"${CMAKE_SOURCE_DIR}/Middlewares/*.c"
    "${CMAKE_SOURCE_DIR}/Src/*.c"
	"${CMAKE_SOURCE_DIR}/startup/*.s"	
)

target_sources(${PROJECT_NAME}.elf PRIVATE ${_source_list})

include_directories (		
    Inc
	Drivers/STM32F4xx_HAL_Driver/Inc	
	Middlewares/ST/STM32_USB_Device_Library/Core/Inc
	Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc
	Drivers/CMSIS/Device/ST/STM32F4xx/Include
	Drivers/CMSIS/Include
	Middlewares/Third_Party/FreeRTOS/Source/include	
	Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407VGTx_FLASH.ld)

target_link_options(${PROJECT_NAME}.elf
    PRIVATE
    -T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,-Map=${PROJECT_NAME}.map)
target_link_libraries(${PROJECT_NAME}.elf m)  
set_target_properties(${PROJECT_NAME}.elf PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})


add_custom_target( compiler_version ALL
  COMMAND
    ${CMAKE_C_COMPILER} --version 
  )

add_custom_target(${PROJECT_NAME}.bin ALL
    COMMAND
      ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
  )
add_dependencies(${PROJECT_NAME}.bin ${PROJECT_NAME}.elf)
