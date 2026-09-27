find_package(Protobuf REQUIRED)
find_package(openocean_messages 0.1 REQUIRED COMPONENTS cpp)

add_library(ivpico_interface)
protobuf_generate(
  TARGET ivpico_interface
  LANGUAGE cpp
  PROTOS interface/options.proto interface/ivp.proto
  IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${openocean_messages_PROTO_DIR}
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(ivpico_interface PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(ivpico_interface PUBLIC openocean_messages::openocean_messages)
