find_package(Doxygen 1.13.2 REQUIRED)

# use Doxygen Awesome stylesheets for doxygen content
include(FetchContent)
FetchContent_Declare(
    doxygen-awesome-css
    URL https://github.com/jothepro/doxygen-awesome-css/archive/refs/heads/main.zip
    GIT_TAG v2.4.1
)
FetchContent_MakeAvailable(doxygen-awesome-css)
FetchContent_GetProperties(doxygen-awesome-css SOURCE_DIR AWESOME_CSS_DIR)

# SDK include path allows the preprocessor to resolve types and header paths
get_filename_component(CPP_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." REALPATH)
set(CPP_SDK_INCLUDE "${CPP_SDK_DIR}/include")
#set(CPP_SDK_EXAMPLES "${CPP_SDK_DIR}/examples")

# runtime include path should be relative to this project, which will resolve
# any references to runtime header files that would be installed on the user system
if(EXISTS "${CPP_SDK_DIR}/../lib/runtime/include")
    get_filename_component(RUNTIME_INCLUDE_DIR "${CPP_SDK_DIR}/../lib/runtime/include" REALPATH)
endif()
if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/../_deps/xronos-lib-build/runtime/include")
    get_filename_component(RUNTIME_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/../_deps/xronos-lib-build/runtime/include" REALPATH)    
endif()

# doxygen INPUT
set(DOXYGEN_INPUT_LIST
    "${CPP_SDK_INCLUDE}"
    "${CPP_SDK_EXAMPLES}"
    "${CMAKE_CURRENT_SOURCE_DIR}/API.md"
)
string(JOIN "\" \"" DOXYGEN_INPUT ${DOXYGEN_INPUT_LIST})
set(DOXYGEN_INPUT "\"${DOXYGEN_INPUT}\"")
message(STATUS "Doxygen inputs: ${DOXYGEN_INPUT}")

# doxygen INCLUDE paths are not necessarily inputs but may be used to resolve includes
# include relative path to runtime when building from source
set(DOXYGEN_INCLUDE_LIST
    "${CPP_SDK_INCLUDE}"
    "${RUNTIME_INCLUDE_DIR}"
    "${RUNTIME_GEN_DIR}"
)
string(JOIN "\" \"" DOXYGEN_INCLUDE ${DOXYGEN_INCLUDE_LIST})
set(DOXYGEN_INCLUDE "\"${DOXYGEN_INCLUDE}\"")
message(STATUS "Doxygen includes: ${DOXYGEN_INCLUDE}")

# configure clang options for Doxygen
set(DOXYGEN_CLANG_ASSISTED_PARSING YES)
set(DOXYGEN_CLANG_DATABASE_PATH "${CMAKE_CURRENT_BINARY_DIR}/../") # path to compile_commands.json
## clang options -- add doxygen includes to include path
string(JOIN "\" -I\"" DOXYGEN_CLANG_OPTIONS ${DOXYGEN_INCLUDE_LIST})
set(DOXYGEN_CLANG_OPTIONS "-I\"${DOXYGEN_CLANG_OPTIONS}\"")
message(STATUS "Doxygen clang options: ${DOXYGEN_CLANG_OPTIONS}")

set(DOXYFILE "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${DOXYFILE} @ONLY)

add_custom_target(docs
    COMMAND Doxygen::doxygen ${DOXYFILE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating C++ API documentation with Doxygen"
    VERBATIM
)
