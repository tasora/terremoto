CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(terremoto)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(ChronoEngine COMPONENTS unit_POSTPROCESS unit_IRRLICHT)

include_directories(${CHRONOENGINE_INCLUDES})

add_executable(myexe terremoto.cpp)

target_link_libraries(myexe ${CHRONOENGINE_LIBRARIES})