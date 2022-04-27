cmake_minimum_required(VERSION 3.5)

set(ESP32_BUTTON_ROOT ../../..)
set(EXTRA_COMPONENT_DIRS  ${ESP32_BUTTON_ROOT})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(interatctive-test)