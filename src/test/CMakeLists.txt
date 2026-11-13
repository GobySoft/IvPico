add_executable(check_helm_libs check_helm_libs.cpp)
target_link_libraries(check_helm_libs PRIVATE IVP::helmivp IVP::ivpbuild IVP::ivpsolve IVP::ivpcore)
add_test(NAME helm_libs COMMAND check_helm_libs)

add_executable(test_helm helm/test_helm.cpp)
target_link_libraries(test_helm PRIVATE ivpico_helm IVP::contacts IVP::geometry IVP::mbutil)
add_test(NAME helm COMMAND test_helm)

add_executable(test_behavior_file helm/test_behavior_file.cpp)
target_link_libraries(test_behavior_file PRIVATE ivpico_helm)
add_test(NAME behavior_file
  COMMAND test_behavior_file ${CMAKE_CURRENT_SOURCE_DIR}/missions/fixtures)

add_test(NAME bhv_schema
  COMMAND ${CMAKE_COMMAND} -DTOOL=$<TARGET_FILE:ivpico_bhv>
          -DSCHEMA=${PROJECT_SOURCE_DIR}/share/ivpico/bhv.schema.json -P ${CMAKE_CURRENT_SOURCE_DIR}/bhv/check_schema.cmake)

# validating against the schema needs python3-jsonschema and python3-yaml
find_package(Python3 COMPONENTS Interpreter QUIET)
if(enable_yaml AND Python3_Interpreter_FOUND)
  execute_process(COMMAND ${Python3_EXECUTABLE} -c "import jsonschema, yaml"
    RESULT_VARIABLE python_modules_missing OUTPUT_QUIET ERROR_QUIET)
  if(python_modules_missing)
    message(STATUS "Skipping the bhv_schema_examples test: needs python3-jsonschema and python3-yaml")
  else()
    add_test(NAME bhv_schema_examples
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/bhv/validate_schema.py
              ${PROJECT_SOURCE_DIR}/share/ivpico/bhv.schema.json $<TARGET_FILE:ivpico_bhv>
              ${CMAKE_CURRENT_SOURCE_DIR}/missions/fixtures
              ${PROJECT_SOURCE_DIR}/launch/goby/alpha/config/alpha.bhv.yml)
  endif()
endif()

add_executable(replay missions/replay.cpp)
# conversions.h includes MOOS-IvP headers
target_link_libraries(replay PRIVATE ivpico_helm IVP::contacts IVP::geometry IVP::mbutil)
file(GLOB fixtures LIST_DIRECTORIES true ${CMAKE_CURRENT_SOURCE_DIR}/missions/fixtures/*)
foreach(fixture ${fixtures})
  if(IS_DIRECTORY ${fixture})
    get_filename_component(mission ${fixture} NAME)
    add_test(NAME replay_${mission} COMMAND replay ${fixture})
    if(enable_yaml)
      add_test(NAME replay_${mission}_yaml COMMAND replay ${fixture} --via-yaml)
    endif()
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

  set(closed_loop_missions s1_alpha s2_bravo s3_charlie s4_delta m2_berta s1_alpha_inline)
  if(enable_yaml)
    list(APPEND closed_loop_missions s1_alpha_yaml)
  endif()
  set(simulator_port 54400)
  foreach(mission ${closed_loop_missions})
    add_test(NAME closed_loop_${mission}
      COMMAND ${GOBY_EXECUTABLE} test run ${CMAKE_CURRENT_SOURCE_DIR}/missions/goby/${mission}/test.yml
              ${goby_test_defines} -D MISSION=${mission} -D PORT=${simulator_port})
    set_tests_properties(closed_loop_${mission} PROPERTIES TIMEOUT 180)
    math(EXPR simulator_port "${simulator_port} + 1")
  endforeach()
elseif(GOBY_FOUND)
  message(STATUS "Skipping the Goby tests: this Goby has no 'goby test' (needs a build after 3.6.1)")
endif()
