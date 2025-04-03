add_library(xronos-graph-messages xronos/messages/reactor_graph.proto
                                  xronos/messages/source_info.proto
                                  xronos/services/diagram_generator.proto)
target_include_directories(
  xronos-graph-messages
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
         "$<INSTALL_INTERFACE:include>")
target_link_libraries(xronos-graph-messages PUBLIC protobuf::libprotobuf
                                                   gRPC::grpc gRPC::grpc++)
set_target_properties(xronos-graph-messages PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)

protobuf_generate(TARGET xronos-graph-messages LANGUAGE cpp)
protobuf_generate(
  TARGET
  xronos-graph-messages
  LANGUAGE
  grpc
  GENERATE_EXTENSIONS
  .grpc.pb.h
  .grpc.pb.cc
  PLUGIN
  "protoc-gen-grpc=${grpc_cpp_plugin_location}")

if(XRONOS_LIB_INSTALL_HEADERS)
  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/xronos" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
endif()

add_library(xronos::xronos-graph-messages ALIAS xronos-graph-messages)
