cmake_minimum_required(VERSION 3.6)

project(vl53lXx CXX)

set (CMAKE_CXX_STANDARD 11)

set(VL53LXX_MAJOR_VERSION 0)
set(VL53LXX_MINOR_VERSION 1)
set(VL53LXX_PATCH_VERSION 1)
set(VL53LXX_VERSION
  ${VL53LXX_MAJOR_VERSION}.${VL53LXX_MINOR_VERSION}.${VL53LXX_PATCH_VERSION})

#Define cmake install path
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

include_directories(
                      include/
                      ${CMAKE_CURRENT_SOURCE_DIR}
                      ${CMAKE_CURRENT_BINARY_DIR}
                   )

set(PROJECT_SRC src/vl53lxx.cpp src/vl53l0x.cpp src/vl53l1x.cpp)

#if(BUILD_I2C_DEV)
MESSAGE("** Compile with i2c dev support.")
add_definitions(-DBUILD_I2C_DEV=ON)
set(PROJECT_SRC ${PROJECT_SRC} src/interfaces/i2cdev.cpp)
#endif()

add_library(${PROJECT_NAME} SHARED ${PROJECT_SRC})

set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${VL53LXX_VERSION})

#target_link_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/vl53lXx/
  DESTINATION include/vl53lXx)

IF(WITH_TEST)
  find_package(Boost REQUIRED COMPONENTS program_options)
  include_directories(${Boost_INCLUDE_DIRS})

  add_executable(vl53lXx_run test/vl53lXx_run.cpp)
  target_link_libraries(${PROJECT_NAME}_run ${PROJECT_NAME} ${Boost_LIBRARIES})

  install(TARGETS vl53lXx_run
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
ENDIF(WITH_TEST)

# Handle CMake Config
# Add all targets to the build-tree export set
export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${PROJECT_NAME})

# Create the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}ConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}/include")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${VL53LXX_CMAKE_DIR}/${REL_INCLUDE_DIR}/${PROJECT_NAME}")
configure_file(${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)
# ... for both
configure_file(${PROJECT_NAME}ConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)

# Install the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}ConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
# install(EXPORT ${PROJECT_NAME}Targets DESTINATION
#  "${INSTALL_CMAKE_DIR}" COMPONENT dev)
