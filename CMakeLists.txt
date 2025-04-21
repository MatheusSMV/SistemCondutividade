# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico CACHE STRING "Board type")
include(pico_sdk_import.cmake)

# Definir o Nome do Projeto
project(Projeto_01 C CXX ASM)
pico_sdk_init()

add_executable(${PROJECT_NAME}
Projeto_01.c # Nome do Projeto Principal em C
inc/ssd1306.c # Biblioteca para o display OLED
)

# Bibliotecas Utilizadas
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_i2c
    hardware_adc
    hardware_pwm
    hardware_gpio
    hardware_pio
)

# Habilitando UART
pico_enable_stdio_uart(${PROJECT_NAME} 0)
pico_enable_stdio_usb(${PROJECT_NAME} 1)


target_include_directories(${PROJECT_NAME} PRIVATE 
  ${PICO_SDK_PATH}/src/rp2_common/hardware_pio/include
)

pico_add_extra_outputs(${PROJECT_NAME})