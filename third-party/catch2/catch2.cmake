set(CATCH2_VERSION "3.9.1" CACHE STRING "Catch2 version")
option(CATCH2_FORCE_BUILD "Force building Catch2 from source" OFF)
option(CATCH2_INSTALL "Add Catch2 to the install target" OFF)

if(${CATCH2_INSTALL})
  # trick Catch2 into thinking that it is in the top level
  unset(PROJECT_NAME)
else()
  set(CATCH2_EXCLUDE_FROM_ALL EXCLUDE_FROM_ALL)
endif()

if(NOT ${CATCH2_FORCE_BUILD})
  set(CATCH2_FIND_PACKAGE_ARGS FIND_PACKAGE_ARGS ${CATCH2_VERSION} EXACT CONFIG)
endif()

include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG "v${CATCH2_VERSION}"
  ${CATCH2_EXCLUDE_FROM_ALL}
  ${CATCH2_FIND_PACKAGE_ARGS}
)
FetchContent_MakeAvailable(Catch2)

if(DEFINED catch2_SOURCE_DIR)
  list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
endif()
