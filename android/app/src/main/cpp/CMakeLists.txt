cmake_minimum_required(VERSION 3.22)
project(psom_android CXX)

# The engine, built for the phone against the deps from engine/scripts/build-android-deps.sh.
set(PSOM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../../..)
set(DEPS ${PSOM_ROOT}/third_party/android)
if(NOT EXISTS ${DEPS}/occt/lib/cmake/opencascade/OpenCASCADEConfig.cmake)
  message(FATAL_ERROR "Android deps missing: run engine/scripts/build-android-deps.sh first")
endif()
set(OpenCV_STATIC ON)
set(OpenCV_DIR ${DEPS}/opencv/sdk/native/jni)
set(OpenCASCADE_DIR ${DEPS}/occt/lib/cmake/opencascade)
set(PSOM_BUILD_CLI OFF CACHE BOOL "" FORCE)
set(PSOM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${PSOM_ROOT}/engine engine)

add_library(psom SHARED jni.cpp)
target_link_libraries(psom PRIVATE psom_engine nlohmann_json::nlohmann_json log)
