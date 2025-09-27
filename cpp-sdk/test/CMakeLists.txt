file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")
set(TEST_SOURCES ${TEST_SOURCES} PARENT_SCOPE)  # make the variable available in parent CMakeLists.txt

if(XRONOS_SDK_BUILD_TESTS)
  include(../../third-party/googletest/googletest.cmake)
  add_executable(xronos-sdk-tests ${TEST_SOURCES})
  target_link_libraries(xronos-sdk-tests PRIVATE xronos::xronos-sdk GTest::gtest_main xronos::xronos-util)
  include(GoogleTest)
  gtest_discover_tests(xronos-sdk-tests)
endif()
