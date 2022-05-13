# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{MDF_PATH}/project.cmake)

project(ESP32Firmware)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# For adding the PEM to the MQTT
#target_add_binary_data(ESP32Firmware.elf "components/mesh_mqtt_handle/mqtt_energysmeter_com.pem" TEXT)