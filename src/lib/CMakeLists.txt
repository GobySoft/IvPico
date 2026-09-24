find_package(Protobuf REQUIRED)

add_library(ivpico_interface)
protobuf_generate(
  TARGET ivpico_interface
  LANGUAGE cpp
  PROTOS interface/options.proto interface/ivp.proto
  IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(ivpico_interface PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(ivpico_interface PUBLIC protobuf::libprotobuf)
