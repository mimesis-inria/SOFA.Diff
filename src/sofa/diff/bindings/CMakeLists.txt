project(Bindings_SOFA.Diff)

set(HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_GradientBasedOptimizationLoop.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_OptimizationLoop.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_Parameter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_ParameterizedForceField.h
)

set(SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_GradientBasedOptimizationLoop.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_OptimizationLoop.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_Parameter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_ParameterizedForceField.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Module_SOFA.Diff.cpp
)

#find_package(SOFA.Diff QUIET REQUIRED)
sofa_find_package(SOFA.Diff REQUIRED)

if (NOT TARGET SofaPython3::Plugin)
    sofa_find_package(SofaPython3 REQUIRED COMPONENTS Plugin Bindings.Sofa)
endif()

SP3_add_python_module(
        TARGET       ${PROJECT_NAME}
        PACKAGE      sofadiff
        MODULE       sofadiff
        SOURCES      ${SOURCE_FILES}
        HEADERS      ${HEADER_FILES}
        DEPENDS      SOFA.Diff SofaPython3::Plugin SofaPython3::Bindings.Sofa
)

get_target_property(DIRS Bindings_SOFA.Diff INCLUDE_DIRECTORIES)
message(STATUS "Bindings_SOFA.Diff include dirs: ${DIRS}")

message("-- SofaPython3 bindings for SOFA.Diff will be created.")