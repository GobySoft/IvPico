# a test app, so not goby_add_app, which installs it
add_executable(ivpico_ghost ghost.cpp)
protobuf_generate(
  TARGET ivpico_ghost
  LANGUAGE cpp
  PROTOS ghost_config.proto
  IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${GOBY_INCLUDE_DIR} ${GOBY_DCCL_PROTO_DIR}
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(ivpico_ghost PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR}/src/bin/goby)
target_link_libraries(ivpico_ghost goby goby_zeromq ivpico_interface)
