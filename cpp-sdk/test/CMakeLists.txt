file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")
set(TEST_SOURCES ${TEST_SOURCES} PARENT_SCOPE)  # make the variable available in parent CMakeLists.txt

if(XRONOS_SDK_BUILD_TESTS)
  include(../../third-party/googletest/googletest.cmake)
  add_executable(xronos-sdk-tests ${TEST_SOURCES})
  # xronos-runtime-interfaces provides the complete runtime::Runtime type,
  # needed by runtime_provider.cc to implement a RuntimeProvider.
  target_link_libraries(xronos-sdk-tests PRIVATE xronos::xronos-sdk GTest::gtest_main xronos::xronos-runtime-interfaces)
  include(GoogleTest)
  gtest_discover_tests(xronos-sdk-tests)

  # Regression guard: the shared library must export its compiled SDK
  # surface (most importantly the program-context factory) and no strong
  # symbols of the statically linked internal libraries. Only meaningful
  # for the shared build on Linux.
  if(XRONOS_SDK_BUILD_SHARED AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_test(
      NAME xronos-sdk-exported-symbols
      COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/check-exported-symbols.sh $<TARGET_FILE:xronos-sdk>
    )
  endif()
endif()
