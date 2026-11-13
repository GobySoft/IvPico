add_executable(ivpico_bhv ivpico_bhv.cpp)
target_include_directories(ivpico_bhv PRIVATE ${PROJECT_SOURCE_DIR}/src/lib)
target_link_libraries(ivpico_bhv PRIVATE ivpico_helm)
install(TARGETS ivpico_bhv RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES ${PROJECT_SOURCE_DIR}/share/ivpico/bhv.schema.json
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ivpico)
