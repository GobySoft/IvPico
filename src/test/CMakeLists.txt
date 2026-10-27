add_executable(check_helm_libs check_helm_libs.cpp)
target_link_libraries(check_helm_libs PRIVATE IVP::helmivp IVP::ivpbuild IVP::ivpsolve IVP::ivpcore)
add_test(NAME helm_libs COMMAND check_helm_libs)

add_executable(test_helm helm/test_helm.cpp)
target_link_libraries(test_helm PRIVATE ivpico_helm)
add_test(NAME helm COMMAND test_helm)

add_executable(replay missions/replay.cpp)
target_link_libraries(replay PRIVATE ivpico_helm)
file(GLOB fixtures LIST_DIRECTORIES true ${CMAKE_CURRENT_SOURCE_DIR}/missions/fixtures/*)
foreach(fixture ${fixtures})
  if(IS_DIRECTORY ${fixture})
    get_filename_component(mission ${fixture} NAME)
    add_test(NAME replay_${mission} COMMAND replay ${fixture})
  endif()
endforeach()

if(GOBY_FOUND)
  add_test(NAME goby_helm_ivp
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/goby/test_goby_helm_ivp.sh
            $<TARGET_FILE:goby_helm_ivp> $<TARGET_FILE:ivpico_interface>)
endif()
