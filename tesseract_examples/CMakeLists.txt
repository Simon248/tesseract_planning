cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_task_composer REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(
  tesseract_motion_planners REQUIRED
  COMPONENTS core simple
  OPTIONAL_COMPONENTS
    descartes
    ompl
    trajopt
    trajopt_ifopt)
find_package(tesseract_collision REQUIRED COMPONENTS core)
find_package(
  PCL REQUIRED
  COMPONENTS core
             features
             filters
             io
             segmentation
             surface)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(
  ${PROJECT_NAME}
  src/example.cpp
  src/basic_cartesian_example.cpp
  src/car_seat_example.cpp
  src/freespace_hybrid_example.cpp
  src/freespace_ompl_example.cpp
  src/glass_upright_example.cpp
  src/online_planning_example.cpp
  src/pick_and_place_example.cpp
  src/puzzle_piece_auxillary_axes_example.cpp
  src/puzzle_piece_example.cpp
  src/scene_graph_example.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  tesseract::tesseract_environment
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_trajopt_ifopt
  trajopt::trajopt_sqp
  trajopt::trajopt_ifopt
  tesseract::tesseract_task_composer
  tesseract::tesseract_task_composer_planning_nodes
  console_bridge::console_bridge
  ${PCL_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
foreach(DEF ${PCL_DEFINITIONS})
  string(STRIP ${DEF} DEF)
  if(NOT
     "${DEF}"
     STREQUAL
     "")
    string(
      SUBSTRING "${DEF}"
                0
                2
                DEF_PREFIX)
    if("${DEF_PREFIX}" STREQUAL "-m")
      string(
        REPLACE " "
                ";"
                DEF
                ${DEF})
      foreach(OPTION_DEF ${DEF})
        target_compile_options(${PROJECT_NAME} PUBLIC ${OPTION_DEF})
      endforeach()
    else()
      target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF})
    endif()
  endif()
endforeach()
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})

macro(add_example example_name example_file)
  add_executable(${example_name} ${example_file})
  target_link_libraries(${example_name} ${PROJECT_NAME})
  target_compile_options(${example_name} PRIVATE ${TESSERACT_COMPILE_OPTIONS})
  target_clang_tidy(${example_name} ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
  target_cxx_version(${example_name} PUBLIC VERSION ${TESSERACT_CXX_VERSION})
  target_include_directories(${example_name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                     "$<INSTALL_INTERFACE:include>")
endmacro()

add_example(${PROJECT_NAME}_basic_cartesian_example src/basic_cartesian_example_node.cpp)
add_example(${PROJECT_NAME}_car_seat_example src/car_seat_example_node.cpp)
add_example(${PROJECT_NAME}_freespace_hybrid_example src/freespace_hybrid_example_node.cpp)
add_example(${PROJECT_NAME}_freespace_ompl_example src/freespace_ompl_example_node.cpp)
add_example(${PROJECT_NAME}_glass_upright_example src/glass_upright_example_node.cpp)
add_example(${PROJECT_NAME}_online_planning_example src/online_planning_example_node.cpp)
add_example(${PROJECT_NAME}_pick_and_place_example src/pick_and_place_example_node.cpp)
add_example(${PROJECT_NAME}_puzzle_piece_auxillary_axes_example src/puzzle_piece_auxillary_axes_example_node.cpp)
add_example(${PROJECT_NAME}_puzzle_piece_example src/puzzle_piece_example_node.cpp)
add_example(${PROJECT_NAME}_scene_graph_example src/scene_graph_example_node.cpp)

# OMPL Constraint planning is not available in version less than 1.4 if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
# add_example(${PROJECT_NAME}_glass_upright_ompl_example src/glass_upright_ompl_example_node.cpp) endif()

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}
          ${PROJECT_NAME}_basic_cartesian_example
          ${PROJECT_NAME}_car_seat_example
          ${PROJECT_NAME}_freespace_hybrid_example
          ${PROJECT_NAME}_freespace_ompl_example
          ${PROJECT_NAME}_glass_upright_example
          ${PROJECT_NAME}_online_planning_example
          ${PROJECT_NAME}_pick_and_place_example
          ${PROJECT_NAME}_puzzle_piece_auxillary_axes_example
          ${PROJECT_NAME}_puzzle_piece_example
          ${PROJECT_NAME}_scene_graph_example
  DEPENDENCIES
    tesseract_environment
    tesseract_command_language
    tesseract_motion_planners
    tesseract_task_composer
    tesseract_collision
    tesseract_common
    trajopt_sqp
    trajopt_ifopt
    "PCL REQUIRED COMPONENTS core features filters io segmentation surface")

if(TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${TESSERACT_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

if(TESSERACT_PACKAGE)
  cpack(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    LINUX_DEPENDS
      "libconsole-bridge-dev"
      "libpcl-dev"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-ompl"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-trajopt"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-trajopt-ifopt"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-task-composer-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-task-composer-planning"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-task-composer-taskflow"
      "${TESSERACT_PACKAGE_PREFIX}trajopt-sqp"
      "${TESSERACT_PACKAGE_PREFIX}trajopt-ifopt"
    WINDOWS_DEPENDS
      "console-bridge"
      "pcl"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-common"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-collision-bullet"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-environment"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-command-language"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-ompl"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-trajopt"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-motion-planners-trajopt-ifopt"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-task-composer-core"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-task-composer-planning"
      "${TESSERACT_PACKAGE_PREFIX}tesseract-task-composer-taskflow"
      "${TESSERACT_PACKAGE_PREFIX}trajopt-sqp"
      "${TESSERACT_PACKAGE_PREFIX}trajopt-ifopt")

  if(UNIX AND TESSERACT_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${TESSERACT_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${TESSERACT_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${TESSERACT_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${TESSERACT_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
