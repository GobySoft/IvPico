add_subdirectory(helm_ivp)

if(GOBY_moos_FOUND AND MOOS_FOUND)
  add_subdirectory(moos_gateway)
else()
  message(STATUS "Goby's MOOS support not found: not building the goby_moos_gateway plugin")
endif()
