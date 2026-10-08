#
# Mbed CE Combined Project: LVGL + MQTT
#
cmake_minimum_required(VERSION 3.19)
cmake_policy(VERSION 3.19...3.22)

### Init Mbed OS ###
######################################################################################################
set(MBED_APP_JSON_PATH mbed_app.json) # 如果用的是 json 請改成 mbed_app.json

include(mbed-os/tools/cmake/mbed_toolchain_setup.cmake)

project(lv_F769 LANGUAGES C CXX ASM)

include(mbed_project_setup)

######################################################################################################
### Subdirectories

# Uncomment if you have custom_targets
# add_subdirectory(custom_targets)

add_subdirectory(mbed-os)
add_subdirectory(lvgl)
add_subdirectory(components/paho_mqtt/MQTTPacket)

add_library(paho_mqtt STATIC
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTConnectClient.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTConnectServer.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTDeserializePublish.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTSerializePublish.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTFormat.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTPacket.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTSubscribeClient.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTSubscribeServer.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTUnsubscribeClient.c
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket/MQTTUnsubscribeServer.c
)

######################################################################################################
### Include Directories

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/lvgl
    ${CMAKE_SOURCE_DIR}/Utilities/STM32F769I-Discovery
    ${CMAKE_SOURCE_DIR}/Utilities/Components/ft6x06/
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTClient
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTClient/mbed
    ${CMAKE_SOURCE_DIR}/components/paho_mqtt/MQTTPacket
)

link_directories(${CMAKE_SOURCE_DIR})

######################################################################################################
### Source Files

file(GLOB_RECURSE HAL_Sources 
    Utilities/Components/ft6x06/ft6x06.c
    Utilities/Components/otm8009a/otm8009a.c
    Utilities/STM32F769I-Discovery/stm32f769i_discovery*.c
)

file(GLOB_RECURSE LVGL_Sources CONFIGURE_DEPENDS 
    hal_stm_lvgl/tft.cpp
    hal_stm_lvgl/touchpad.cpp
)

set(MAIN_SOURCES
    src/main.cpp
    ${HAL_Sources}
    ${LVGL_Sources}
)

######################################################################################################
### Definitions

add_definitions(
    -DSTM32
    -DSTM32F7
    -DSTM32F769xx
    -DSTM32F769I_DISCO
    -DDEBUG
    -DUSE_HAL_DRIVER
    -DDMA2_Stream0=0
)

######################################################################################################
### Executable

add_executable(${PROJECT_NAME} ${MAIN_SOURCES})

######################################################################################################
### Linking Libraries

if("MBED_CONF_TARGET_APPLICATION_PROFILE=bare-metal" IN_LIST MBED_CONFIG_DEFINITIONS)
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            m
            lvgl
            lvgl_demos
            mbed-baremetal
            paho_mqtt
            mbed-netsocket
            mbed-mbedtls
    )
else()
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            m
            lvgl
            lvgl_demos
            mbed-os
            paho_mqtt
            mbed-netsocket
            mbed-mbedtls
    )
endif()

mbed_set_post_build(${PROJECT_NAME})
