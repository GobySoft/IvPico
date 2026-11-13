add_library(ivpico_interface)
protobuf_generate(
  TARGET ivpico_interface
  LANGUAGE cpp
  PROTOS interface/options.proto interface/ivp.proto interface/config.proto
  IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${openocean_messages_PROTO_DIR}
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(ivpico_interface PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(ivpico_interface PUBLIC openocean_messages::openocean_messages)

add_library(ivpico_helm helm/helm.cpp helm/conversions.cpp helm/status_strings.cpp
  helm/contact_manager.cpp helm/contacts/CMAlert.cpp helm/contacts/PlatformAlertRecord.cpp)
target_include_directories(ivpico_helm PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(ivpico_helm
  PUBLIC ivpico_interface
  PRIVATE IVP::helmivp IVP::geodaid IVP::behaviors IVP::contacts IVP::logic IVP::ivpbuild
          IVP::ivpsolve IVP::ivpcore IVP::geometry IVP::mbutil)

install(TARGETS ivpico_interface ivpico_helm
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
