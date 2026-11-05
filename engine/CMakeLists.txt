cmake_minimum_required(VERSION 3.22)
project(psom LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(PSOM_BUILD_CLI "Build the psom command-line tool" ON)
option(PSOM_BUILD_TESTS "Build the tests" ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(OpenCASCADE REQUIRED)

include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(json)

add_library(psom_engine STATIC
  src/session.cpp src/depth.cpp src/mvs.cpp src/isolate.cpp src/tsdf.cpp src/mesh.cpp
  src/repair.cpp src/step.cpp src/pipeline.cpp src/synth.cpp)
target_include_directories(psom_engine PUBLIC include ${OpenCASCADE_INCLUDE_DIR})
target_link_libraries(psom_engine
  PUBLIC opencv_core opencv_imgproc opencv_imgcodecs
  PRIVATE nlohmann_json::nlohmann_json
          TKDESTEP TKXSBase TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel)
set_target_properties(psom_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(PSOM_BUILD_CLI)
  add_executable(psom cli/main.cpp)
  target_link_libraries(psom PRIVATE psom_engine nlohmann_json::nlohmann_json)
endif()

if(PSOM_BUILD_TESTS)
  enable_testing()
  FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  file(GLOB PSOM_TEST_SOURCES CONFIGURE_DEPENDS test/test_*.cpp)
  add_executable(psom_tests ${PSOM_TEST_SOURCES})
  target_link_libraries(psom_tests PRIVATE psom_engine nlohmann_json::nlohmann_json GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(psom_tests DISCOVERY_TIMEOUT 60 PROPERTIES TIMEOUT 900)
endif()
