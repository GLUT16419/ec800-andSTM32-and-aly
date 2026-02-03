cmake_minimum_required(VERSION 3.16)

project(dgb C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# 定义目标芯片
set(STM32_CHIP STM32F103C8T6)

# 包含STM32F103C8T6的设备文件
set(STM32_DEVICE_DIR ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include)
set(STM32_CORE_DIR ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include)

# 包含HAL库头文件
set(STM32_HAL_DIR ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc)
set(STM32_HAL_SRC_DIR ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src)

# 包含用户代码头文件
set(USER_INC_DIR ${CMAKE_SOURCE_DIR}/Core/Inc)
set(USER_SRC_DIR ${CMAKE_SOURCE_DIR}/Core/Src)

# 包含所有头文件目录
include_directories(
    ${STM32_DEVICE_DIR}
    ${STM32_CORE_DIR}
    ${STM32_HAL_DIR}
    ${STM32_HAL_DIR}/Legacy
    ${USER_INC_DIR}
)

# 收集所有源文件
file(GLOB_RECURSE USER_SOURCES ${USER_SRC_DIR}/*.c)
file(GLOB_RECURSE HAL_SOURCES ${STM32_HAL_SRC_DIR}/*.c)

# 添加启动文件
set(STARTUP_FILE ${CMAKE_SOURCE_DIR}/startup_stm32f103xb.s)

# 添加链接脚本
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F103XX_FLASH.ld)

# 设置编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -std=gnu11 -Wall -Wextra -Os")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -std=gnu++17 -Wall -Wextra -Os")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -x assembler-with-cpp")

# 设置链接选项
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT} -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -specs=nano.specs -specs=nosys.specs -Wl,-Map=${PROJECT_NAME}.map,--cref -Wl,--gc-sections")

# 创建可执行文件
add_executable(${PROJECT_NAME}.elf
    ${USER_SOURCES}
    ${HAL_SOURCES}
    ${STARTUP_FILE}
)

# 添加后处理命令，生成hex和bin文件
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMENT "Generating hex and bin files..."
)

# 显示大小信息
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Displaying size information..."
)
