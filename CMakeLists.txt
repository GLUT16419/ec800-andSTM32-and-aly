cmake_minimum_required(VERSION 3.13)

project(dgb C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# 定义输出目录
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

# 包含STM32CubeMX生成的文件
add_definitions(-DUSE_HAL_DRIVER -DSTM32F103xB)

# 头文件目录
include_directories(
    Core/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc
    Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F1xx/Include
    Drivers/CMSIS/Include
)

# 源文件目录
file(GLOB_RECURSE SOURCES 
    "Core/Src/*.c"
    "Drivers/STM32F1xx_HAL_Driver/Src/*.c"
    "Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s"
)

# 排除不需要的文件
list(FILTER SOURCES EXCLUDE REGEX "Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_timebase_tim_template.c")

# 创建可执行文件
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# 链接脚本
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103XX_FLASH.ld)
target_link_options(${PROJECT_NAME}.elf PRIVATE -T ${LINKER_SCRIPT})

# 编译选项
target_compile_options(${PROJECT_NAME}.elf PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -mfpu=none
    -mfloat-abi=soft
    -O0
    -g3
    -Wall
    -fmessage-length=0
    -ffunction-sections
    -fdata-sections
)

# 链接选项
target_link_options(${PROJECT_NAME}.elf PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -mfpu=none
    -mfloat-abi=soft
    -Wl,--gc-sections
    -Wl,-Map=${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.map
)

# 生成bin和hex文件
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.hex
    COMMENT "Generating binary and hex files..."
)

# 显示大小
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} ${EXECUTABLE_OUTPUT_PATH}/${PROJECT_NAME}.elf
    COMMENT "Displaying size information..."
)