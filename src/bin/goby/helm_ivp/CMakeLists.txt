goby_add_app(TARGET goby_helm_ivp
  SOURCES helm_ivp.cpp
  PROTOS helm_ivp_config.proto
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto
  PROTO_IMPORT_DIRS ${PROJECT_SOURCE_DIR}/src/lib ${openocean_messages_PROTO_DIR}
  LINK_LIBRARIES goby_zeromq ivpico_helm)
target_include_directories(goby_helm_ivp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
