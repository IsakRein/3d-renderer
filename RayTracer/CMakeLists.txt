project(DH2323Lab2)

cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O2 -Wall")

IF(APPLE)
  SET(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "Build architectures for Mac OS X" FORCE)
ENDIF(APPLE)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
find_package (SDL2 REQUIRED)

message(STATUS "Lib: ${SDL2_LIBRARIES} , Include: ${SDL2_INCLUDE_DIRS}")

include_directories(
  ${SDL2_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/glm
  ${CMAKE_SOURCE_DIR}/SDL2Auxiliary
)

add_executable(Task4
  Task4.cpp
  ${CMAKE_SOURCE_DIR}/SDL2Auxiliary/SDL2Auxiliary.cpp
)

target_link_libraries(Task4
  ${SDL2_LIBRARIES}
)

add_executable(Task5
  Task5.cpp
  ${CMAKE_SOURCE_DIR}/SDL2Auxiliary/SDL2Auxiliary.cpp
)

target_link_libraries(Task5
  ${SDL2_LIBRARIES}
)

add_executable(Task6
  Task6.cpp
  ${CMAKE_SOURCE_DIR}/SDL2Auxiliary/SDL2Auxiliary.cpp
)

target_link_libraries(Task6
  ${SDL2_LIBRARIES}
)