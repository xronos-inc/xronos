include(../../third-party/googletest/googletest.cmake)

add_executable(xronos-sdk-tests
  empty.cc
  loops.cc
  periodic_timers.cc
  ports.cc
  programmable_timers.cc
  startup_and_shutdown.cc
  time.cc
)
target_link_libraries(xronos-sdk-tests PRIVATE xronos-sdk GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(xronos-sdk-tests)
