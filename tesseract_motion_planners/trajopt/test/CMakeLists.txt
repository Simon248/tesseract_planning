find_package(tesseract_command_language REQUIRED)

add_executable(${PROJECT_NAME}_trajopt_unit trajopt_planner_tests.cpp)
target_link_libraries(${PROJECT_NAME}_trajopt_unit PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME}_trajopt)
target_compile_options(${PROJECT_NAME}_trajopt_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE}
                                                            ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_trajopt_unit PRIVATE ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_trajopt_unit ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_trajopt_unit PRIVATE VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_trajopt_unit
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_trajopt_unit)
add_dependencies(${PROJECT_NAME}_trajopt_unit ${PROJECT_NAME}_trajopt)
add_dependencies(run_tests ${PROJECT_NAME}_trajopt_unit)
