file(GLOB_RECURSE EXAMPLE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")
set(EXAMPLE_SOURCES ${EXAMPLE_SOURCES} PARENT_SCOPE)  # make the variable available in parent CMakeLists.txt

if(XRONOS_SDK_BUILD_EXAMPLES)
  # create an executable for each example source file
  foreach(SOURCE ${EXAMPLE_SOURCES})
    get_filename_component(FILENAME ${SOURCE} NAME)
    string(REPLACE ".cc" "" TARGET_NAME ${FILENAME})
    add_executable(${TARGET_NAME} ${SOURCE})
    target_link_libraries(${TARGET_NAME} xronos::xronos-sdk)
  endforeach()
endif()
