add_library(xronos-telemetry lib/attribute_manager.cc)
target_include_directories(
  xronos-telemetry
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")
target_link_libraries(xronos-telemetry PRIVATE xronos-runtime)
if(XRONOS_LIB_RUN_CLANG_TIDY)
  set_target_properties(xronos-telemetry
                        PROPERTIES CXX_CLANG_TIDY "${XRONOS_CLANG_TIDY_COMMAND}")
endif()
set_target_properties(xronos-telemetry PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

if(XRONOS_LIB_INSTALL_HEADERS)
  install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
endif()

add_library(xronos::xronos-telemetry ALIAS xronos-telemetry)
