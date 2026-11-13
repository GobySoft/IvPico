# Fails if the committed schema differs from what ivpico_bhv generates.
# Usage: cmake -DTOOL=<ivpico_bhv> -DSCHEMA=<bhv.schema.json> -P check_schema.cmake
execute_process(COMMAND ${TOOL} schema OUTPUT_VARIABLE generated RESULT_VARIABLE result)
file(READ ${SCHEMA} committed)
if(NOT result EQUAL 0 OR NOT generated STREQUAL committed)
  message(FATAL_ERROR "${SCHEMA} is out of date: regenerate it with\n  ivpico_bhv schema -o ${SCHEMA}")
endif()
