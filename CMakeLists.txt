cmake_minimum_required(VERSION 3.12)
project(SOFA.Diff VERSION 0.1 LANGUAGES CXX)

# Find and load CMake configuration of packages containing this plugin's dependencies
## Mandatory dependencies
sofa_find_package(Sofa.Framework REQUIRED)
sofa_find_package(Sofa.Component.Controller REQUIRED)
sofa_find_package(Sofa.Component.Mapping REQUIRED)
sofa_find_package(Sofa.Component.SolidMechanics.Spring REQUIRED)
sofa_find_package(Sofa.Component.SolidMechanics.FEM.Elastic REQUIRED)


# List all files
set(SOFA_DIFF_SRC_DIR src/sofa/diff)
set(HEADER_FILES
        ${SOFA_DIFF_SRC_DIR}/config.h.in
        ${SOFA_DIFF_SRC_DIR}/initSofaDiff.h
        ${SOFA_DIFF_SRC_DIR}/utils.h
        ${SOFA_DIFF_SRC_DIR}/optimizers/OptimizationLoop.h
        ${SOFA_DIFF_SRC_DIR}/optimizers/GridSearchOptimizationLoop.h
        ${SOFA_DIFF_SRC_DIR}/optimizers/GradientBasedOptimizationLoop.h
        ${SOFA_DIFF_SRC_DIR}/optimizers/GradientDescentOptimizationLoop.h
        ${SOFA_DIFF_SRC_DIR}/DifferentiableAnimationLoop.h
        ${SOFA_DIFF_SRC_DIR}/adjoints/AdjointSolver.h
        ${SOFA_DIFF_SRC_DIR}/adjoints/StaticAdjointSolver.h
        ${SOFA_DIFF_SRC_DIR}/ParameterizedForceField.h
        ${SOFA_DIFF_SRC_DIR}/Parameter.h
        ${SOFA_DIFF_SRC_DIR}/Parameter.inl
        ${SOFA_DIFF_SRC_DIR}/LossState.h
        ${SOFA_DIFF_SRC_DIR}/visitors/AdjointResetVisitor.h
        ${SOFA_DIFF_SRC_DIR}/visitors/AdjointSolveVisitor.h
        ${SOFA_DIFF_SRC_DIR}/visitors/MechanicalAccumulateVecDeriv.h
        ${SOFA_DIFF_SRC_DIR}/visitors/MechanicalPropagateVecDeriv.h
        ${SOFA_DIFF_SRC_DIR}/visitors/StoreStateVisitor.h
        ${SOFA_DIFF_SRC_DIR}/visitors/RetrieveStateVisitor.h
        ${SOFA_DIFF_SRC_DIR}/forcefields/ParameterizedRestShapeSpringsForceField.h
        ${SOFA_DIFF_SRC_DIR}/forcefields/ParameterizedSpringForceField.h
        ${SOFA_DIFF_SRC_DIR}/forcefields/ParameterizedTetrahedronFEMForceField.h
        ${SOFA_DIFF_SRC_DIR}/mappings/MeanSquaredErrorMapping.h
)
set(HEADER_FILES_TO_MOC
)
set(SOURCE_FILES
        ${SOFA_DIFF_SRC_DIR}/initSofaDiff.cpp
        ${SOFA_DIFF_SRC_DIR}/optimizers/OptimizationLoop.cpp
        ${SOFA_DIFF_SRC_DIR}/optimizers/GridSearchOptimizationLoop.cpp
        ${SOFA_DIFF_SRC_DIR}/optimizers/GradientBasedOptimizationLoop.cpp
        ${SOFA_DIFF_SRC_DIR}/optimizers/GradientDescentOptimizationLoop.cpp
        ${SOFA_DIFF_SRC_DIR}/DifferentiableAnimationLoop.cpp
        ${SOFA_DIFF_SRC_DIR}/adjoints/AdjointSolver.cpp
        ${SOFA_DIFF_SRC_DIR}/adjoints/StaticAdjointSolver.cpp
        ${SOFA_DIFF_SRC_DIR}/ParameterizedForceField.cpp
        ${SOFA_DIFF_SRC_DIR}/Parameter.cpp
        ${SOFA_DIFF_SRC_DIR}/LossState.cpp
        ${SOFA_DIFF_SRC_DIR}/visitors/AdjointSolveVisitor.cpp
        ${SOFA_DIFF_SRC_DIR}/visitors/AdjointResetVisitor.cpp
        ${SOFA_DIFF_SRC_DIR}/visitors/MechanicalAccumulateVecDeriv.cpp
        ${SOFA_DIFF_SRC_DIR}/visitors/MechanicalPropagateVecDeriv.cpp
        ${SOFA_DIFF_SRC_DIR}/visitors/StoreStateVisitor.cpp
        ${SOFA_DIFF_SRC_DIR}/visitors/RetrieveStateVisitor.cpp
        ${SOFA_DIFF_SRC_DIR}/forcefields/ParameterizedRestShapeSpringsForceField.cpp
        ${SOFA_DIFF_SRC_DIR}/forcefields/ParameterizedSpringForceField.cpp
        ${SOFA_DIFF_SRC_DIR}/forcefields/ParameterizedTetrahedronFEMForceField.cpp
        ${SOFA_DIFF_SRC_DIR}/mappings/MeanSquaredErrorMapping.cpp
)

option(SOFA_DIFF_BUILD_GUI_COMPONENTS "build ImGui dependent code" OFF)
if (SOFA_DIFF_BUILD_GUI_COMPONENTS)
    find_package(SofaImGui REQUIRED)
    list(APPEND HEADER_FILES ${SOFA_DIFF_SRC_DIR}/imgui/LoopsControlsGUI.h)
    list(APPEND SOURCE_FILES ${SOFA_DIFF_SRC_DIR}/imgui/LoopsControlsGUI.cpp)
endif ()

set(README_FILES
        README.md
)


# Create the plugin library.
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES_TO_MOC} ${MOCCED_HEADER_FILES} ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

# Link the plugin library to its dependency(ies).
target_link_libraries(${PROJECT_NAME} Sofa.Framework Sofa.Component.Controller Sofa.Component.Mapping Sofa.Component.SolidMechanics.Spring Sofa.Component.SolidMechanics.FEM.Elastic)

find_package(SofaPython3 QUIET)
if(SofaPython3_FOUND)
    add_subdirectory(src/sofa/diff/bindings/)
endif()
if (SOFA_DIFF_BUILD_GUI_COMPONENTS)
    target_link_libraries(${PROJECT_NAME} SofaImGui)
endif ()

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
        RELOCATABLE "plugins"
)
