add_executable(check_helm_libs check_helm_libs.cpp)
target_link_libraries(check_helm_libs PRIVATE IVP::helmivp IVP::ivpbuild IVP::ivpsolve IVP::ivpcore)
add_test(NAME helm_libs COMMAND check_helm_libs)

add_executable(test_helm helm/test_helm.cpp)
target_link_libraries(test_helm PRIVATE ivpico_helm IVP::contacts IVP::geometry IVP::mbutil)
add_test(NAME helm COMMAND test_helm)

add_executable(replay missions/replay.cpp)
# conversions.h includes MOOS-IvP headers
target_link_libraries(replay PRIVATE ivpico_helm IVP::contacts IVP::geometry IVP::mbutil)
file(GLOB fixtures LIST_DIRECTORIES true ${CMAKE_CURRENT_SOURCE_DIR}/missions/fixtures/*)
foreach(fixture ${fixtures})
  if(IS_DIRECTORY ${fixture})
    get_filename_component(mission ${fixture} NAME)
    add_test(NAME replay_${mission} COMMAND replay ${fixture})
    file(GLOB moos_file ${fixture}/*.moos)
    file(STRINGS ${moos_file} contact_manager REGEX "ProcessConfig *= *pContactMgrV20")
    if(contact_manager)
      add_test(NAME replay_${mission}_alerts COMMAND replay ${fixture} --alerts)
    endif()
  endif()
endforeach()

# 'goby test' is in Goby builds after 3.6.1, which set GOBY_EXECUTABLE
if(GOBY_FOUND AND GOBY_EXECUTABLE)
  add_subdirectory(goby/ghost)
  set(goby_test_defines
    -D HELM=$<TARGET_FILE:goby_helm_ivp> -D INTERFACE_LIB=$<TARGET_FILE:ivpico_interface>
    -D GHOST=$<TARGET_FILE:ivpico_ghost>)

  add_test(NAME goby_helm_ivp
    COMMAND ${GOBY_EXECUTABLE} test run ${CMAKE_CURRENT_SOURCE_DIR}/goby/helm_ivp/test.yml
            ${goby_test_defines})

  set(simulator_port 54400)
  foreach(mission s1_alpha s2_bravo s3_charlie s4_delta m2_berta)
    add_test(NAME closed_loop_${mission}
      COMMAND ${GOBY_EXECUTABLE} test run ${CMAKE_CURRENT_SOURCE_DIR}/missions/goby/${mission}/test.yml
              ${goby_test_defines} -D MISSION=${mission} -D PORT=${simulator_port})
    set_tests_properties(closed_loop_${mission} PROPERTIES TIMEOUT 180)
    math(EXPR simulator_port "${simulator_port} + 1")
  endforeach()
elseif(GOBY_FOUND)
  message(STATUS "Skipping the Goby tests: this Goby has no 'goby test' (needs a build after 3.6.1)")
endif()
