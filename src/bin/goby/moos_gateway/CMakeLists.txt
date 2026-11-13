goby_add_library(TARGET ivpico_moos_gateway_plugin
  SOURCES moos_gateway.cpp
  PROTOS moos_gateway_config.proto
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto
  PROTO_IMPORT_DIRS ${PROJECT_SOURCE_DIR}/src/lib ${openocean_messages_PROTO_DIR}
  LINK_LIBRARIES goby_moos ivpico_helm IVP::contacts IVP::geometry IVP::mbutil)
target_include_directories(ivpico_moos_gateway_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
install(TARGETS ivpico_moos_gateway_plugin LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
