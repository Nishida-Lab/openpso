cmake_minimum_required(VERSION 3.0.2)

# メジャーバージョンの設定。複雑さと使いやすさのバランスをとって手動更新としている。
set(MAJOR_VERSION 0)
set(MAJOR_VERSION_COMMIT_OFFSET 0)

set(MINOR_VERSION 0)
set(MINOR_VERSION_COMMIT_OFFSET 0)

execute_process(
  COMMAND git log --oneline COMMAND wc --lines
  OUTPUT_VARIABLE TOTAL_COMMITS
)
math(EXPR SUBMINOR_VERSION
  "${TOTAL_COMMITS} - ${MAJOR_VERSION_COMMIT_OFFSET} - ${MINOR_VERSION_COMMIT_OFFSET}"
)

project(openpso VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${SUBMINOR_VERSION} LANGUAGES CXX)
message(STATUS "OpenPSO ${PROJECT_VERSION}")

set(${PROJECT_NAME}_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(OpenCV 2.4 REQUIRED)

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
if (${PROJECT_NAME}_SOURCES) # 実のところこの条件は必要ない
  foreach(EACH IN LISTS ${PROJECT_NAME}_SOURCES)
    string(REGEX REPLACE "^/(.*/)*(.*).cpp$" "\\2" TARGET ${EACH})
    message(STATUS "${TARGET}")
    add_executable(${TARGET} ${EACH}) # TODO add_library SHARED に変更すること
    target_link_libraries(${TARGET} ${OpenCV_LIBRARIES})
  endforeach()
endif()

