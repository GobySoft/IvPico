add_library(ivpico_interface)
protobuf_generate(
  TARGET ivpico_interface
  LANGUAGE cpp
  PROTOS interface/options.proto interface/ivp.proto interface/behaviors.proto interface/config.proto
  IMPORT_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${openocean_messages_PROTO_DIR}
  PROTOC_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})
target_include_directories(ivpico_interface PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(ivpico_interface PUBLIC openocean_messages::openocean_messages)

add_library(ivpico_helm helm/helm.cpp helm/conversions.cpp helm/status_strings.cpp
  helm/contact_manager.cpp helm/contacts/CMAlert.cpp helm/contacts/PlatformAlertRecord.cpp
  helm/behavior_file.cpp)
target_include_directories(ivpico_helm PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(ivpico_helm
  PUBLIC ivpico_interface
  PRIVATE IVP::helmivp IVP::geodaid IVP::behaviors IVP::contacts IVP::logic IVP::ivpbuild
          IVP::ivpsolve IVP::ivpcore IVP::geometry IVP::mbutil)

if(enable_yaml)
  target_sources(ivpico_helm PRIVATE helm/behavior_yaml.cpp)
  target_compile_definitions(ivpico_helm PUBLIC HAS_YAML)
  if(TARGET yaml-cpp::yaml-cpp)
    target_link_libraries(ivpico_helm PRIVATE yaml-cpp::yaml-cpp)
  else()
    target_link_libraries(ivpico_helm PRIVATE yaml-cpp)
  endif()
endif()

install(TARGETS ivpico_interface ivpico_helm
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
