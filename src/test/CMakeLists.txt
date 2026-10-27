add_executable(check_helm_libs check_helm_libs.cpp)
target_link_libraries(check_helm_libs PRIVATE IVP::helmivp IVP::ivpbuild IVP::ivpsolve IVP::ivpcore)
add_test(NAME helm_libs COMMAND check_helm_libs)

add_executable(test_helm helm/test_helm.cpp)
target_link_libraries(test_helm PRIVATE ivpico_helm)
add_test(NAME helm COMMAND test_helm)
