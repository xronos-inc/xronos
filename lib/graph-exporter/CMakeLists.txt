add_library(xronos-graph-exporter lib/exporter.cc)
target_include_directories(
  xronos-graph-exporter
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")

target_link_libraries(xronos-graph-exporter
                      PRIVATE xronos-runtime
                              xronos-graph-messages)

if(XRONOS_LIB_RUN_CLANG_TIDY)
  set_target_properties(xronos-graph-exporter
                        PROPERTIES CXX_CLANG_TIDY "${XRONOS_CLANG_TIDY_COMMAND}")
endif()
set_target_properties(xronos-graph-exporter PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

if(XRONOS_LIB_BUILD_TESTS)
  add_executable(xronos-graph-exporter-tests test/test.cc)
  target_link_libraries(
    xronos-graph-exporter-tests
    PRIVATE xronos-runtime xronos-graph-exporter xronos-graph-messages
            Catch2::Catch2WithMain)
  catch_discover_tests(xronos-graph-exporter-tests)
endif()

add_library(xronos::xronos-graph-exporter ALIAS xronos-graph-exporter)

if(XRONOS_LIB_INSTALL_HEADERS)
  install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
endif()
