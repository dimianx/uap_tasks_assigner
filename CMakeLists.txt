cmake_minimum_required(VERSION 3.10)

project(UAPTasksAssigner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/include)

add_library(UAPTasksAssigner INTERFACE)
target_include_directories(UAPTasksAssigner INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/uap_ta
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp")

install(TARGETS UAPTasksAssigner EXPORT UAPTasksAssignerTargets)

install(EXPORT UAPTasksAssignerTargets
        FILE UAPTasksAssignerTargets.cmake
        NAMESPACE UAPTasksAssigner::
        DESTINATION share/UAPTasksAssigner/cmake)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/UAPTasksAssignerConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/UAPTasksAssignerConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/UAPTasksAssignerConfig.cmake
    INSTALL_DESTINATION share/UAPTasksAssigner/cmake
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/UAPTasksAssignerConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/UAPTasksAssignerConfigVersion.cmake
    DESTINATION share/UAPTasksAssigner/cmake
)

option(BUILD_TESTS "Build the unit tests" ON)
if(BUILD_TESTS)
  enable_testing()
  include(FetchContent)

  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/release-1.11.0.tar.gz
  )
  FetchContent_MakeAvailable(googletest)
  
  add_executable(test_tasks_assigner test/test_tasks_assigner.cpp)
  target_link_libraries(test_tasks_assigner PRIVATE UAPTasksAssigner GTest::gtest_main)
  
  add_test(NAME test_tasks_assigner COMMAND UAPTasksAssignerTest)
endif()
