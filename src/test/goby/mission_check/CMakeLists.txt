goby_add_app(TARGET ivpico_mission_check
  SOURCES mission_check.cpp
  PROTOS mission_check_config.proto
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto
  PROTO_IMPORT_DIRS ${PROJECT_SOURCE_DIR}/src/lib ${openocean_messages_PROTO_DIR}
  LINK_LIBRARIES goby_zeromq ivpico_interface)
target_include_directories(ivpico_mission_check PRIVATE ${PROJECT_SOURCE_DIR}/src/bin/goby)
