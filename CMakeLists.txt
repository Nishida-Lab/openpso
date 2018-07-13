cmake_minimum_required(VERSION 3.0.2)

# メジャーバージョンの設定。複雑さと使いやすさのバランスをとって手動更新としている。
set(MAJOR_VERSION 0)
set(MAJOR_VERSION_COMMIT_OFFSET 0)

set(MINOR_VERSION 0)
set(MINOR_VERSION_COMMIT_OFFSET 0)

execute_process(COMMAND git log --oneline COMMAND wc --lines OUTPUT_VARIABLE TOTAL_COMMITS)
math(EXPR SUBMINOR_VERSION "${TOTAL_COMMITS} - ${MAJOR_VERSION_COMMIT_OFFSET} - ${MINOR_VERSION_COMMIT_OFFSET}")

project(openpso VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${SUBMINOR_VERSION} LANGUAGES CXX)
message(STATUS "OpenPSO ${PROJECT_VERSION}")

set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++14")
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

set(${PROJECT_NAME}_CONFIGURE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/configure)
if(EXISTS ${${PROJECT_NAME}_CONFIGURE_DIR}/README.md.cmake)
  configure_file(${${PROJECT_NAME}_CONFIGURE_DIR}/README.md.cmake ${CMAKE_CURRENT_SOURCE_DIR}/README.md)
endif()

set(${PROJECT_NAME}_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
if (${PROJECT_NAME}_SOURCES)
  foreach(EACH IN LISTS ${PROJECT_NAME}_SOURCES)
    string(REGEX REPLACE "^/(.*/)*(.*).cpp$" "\\2" TARGET ${EACH})
    message(STATUS "${TARGET}")
    add_executable(${TARGET} ${EACH}) # TODO add_library SHARED に変更すること
    target_link_libraries(${TARGET} ${OpenCV_LIBRARIES})
  endforeach()
endif()

