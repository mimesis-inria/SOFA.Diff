cmake_minimum_required(VERSION 3.12)
project(SofaDiff VERSION 0.1 LANGUAGES CXX)

# Find and load CMake configuration of packages containing this plugin's dependencies
## Mandatory dependencies
find_package(Sofa.Framework REQUIRED)
find_package(Sofa.Component.Controller REQUIRED)
find_package(Sofa.Component.Mapping REQUIRED)

## Optional dependencies
#sofa_find_package(Sofa.GUI.Qt QUIET)
#sofa_find_package(Qt5 COMPONENTS Core QUIET) # Dependency to Qt5Core (needed for qt5_wrap_cpp)

# List all files
set(SOFADIFF_SRC_DIR src/SofaDiff)
set(HEADER_FILES
        ${SOFADIFF_SRC_DIR}/config.h.in
        ${SOFADIFF_SRC_DIR}/GradientDescentController.h
        ${SOFADIFF_SRC_DIR}/ComputeCostGradientVisitor.h
        ${SOFADIFF_SRC_DIR}/CostFunctionMapping.h
        ${SOFADIFF_SRC_DIR}/CostFunctionMapping.inl
        ${SOFADIFF_SRC_DIR}/DistanceToPlaneCost.h
        ${SOFADIFF_SRC_DIR}/DistanceToPlaneCost.inl
)
set(HEADER_FILES_TO_MOC
)
set(SOURCE_FILES
        ${SOFADIFF_SRC_DIR}/initSofaDiff.cpp
        ${SOFADIFF_SRC_DIR}/GradientDescentController.cpp
        ${SOFADIFF_SRC_DIR}/ComputeCostGradientVisitor.cpp
        ${SOFADIFF_SRC_DIR}/CostFunctionMapping.cpp
        ${SOFADIFF_SRC_DIR}/DistanceToPlaneCost.cpp
)
set(README_FILES
        README.md
)



# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES_TO_MOC} ${MOCCED_HEADER_FILES} ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Link the plugin library to its dependency(ies).
target_link_libraries(${PROJECT_NAME} Sofa.Framework Sofa.Component.Controller Sofa.Component.Mapping)
# Link with the optional GUI dependencies.
#if(Qt5Core_FOUND AND Sofa.GUI.Qt_FOUND)
#    target_link_libraries(${PROJECT_NAME} Sofa.GUI.Qt)
#endif()

# Create package Config, Version & Target files.
# Deploy the headers, resources, scenes & examples.
# Set the plugin 'relocatable' if built within SOFA.
# --> see SofaMacros.cmake
sofa_create_package_with_targets(
        PACKAGE_NAME ${PROJECT_NAME}
        PACKAGE_VERSION ${PROJECT_VERSION}
        TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
        INCLUDE_SOURCE_DIR "src"
        INCLUDE_INSTALL_DIR ${PROJECT_NAME}
)

# Tests
# If SOFA_BUILD_TESTS exists and is OFF, then these tests will be auto-disabled
#cmake_dependent_option(SOFADIFF_BUILD_TESTS "Compile the automatic tests" ON "SOFA_BUILD_TESTS OR NOT DEFINED SOFA_BUILD_TESTS" OFF)
#if(SOFADIFF_BUILD_TESTS)
#    enable_testing()
#    add_subdirectory(SofaDiff_test)
#endif()