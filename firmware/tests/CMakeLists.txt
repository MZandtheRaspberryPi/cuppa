cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(lib-tests C CXX ASM)

set( cur_path ${CMAKE_CURRENT_SOURCE_DIR} )
cmake_path(GET cur_path PARENT_PATH parent_path)

add_library(CuppaLib ${parent_path}/src/estimator.cpp ${parent_path}/src/ukf.cpp ${parent_path}/src/utils.cpp)
include_directories(${parent_path}/include/)
target_include_directories(CuppaLib PUBLIC
        ${parent_path}/eigen
        ${parent_path}/include/
)


include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  test_ukf
  test_ukf.cpp
)

target_link_libraries(
  test_ukf
  GTest::gtest_main
  CuppaLib
)

include(GoogleTest)
gtest_discover_tests(test_ukf)