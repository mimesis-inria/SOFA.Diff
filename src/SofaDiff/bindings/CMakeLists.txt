project(Bindings_SofaDiff)

set(HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_GradientBasedOptimizationLoop.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_Parameter.h
)

set(SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_GradientBasedOptimizationLoop.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Binding_Parameter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/Module_SofaDiff.cpp
)

#find_package(SofaDiff QUIET REQUIRED)
sofa_find_package(SofaDiff REQUIRED)

if (NOT TARGET SofaPython3::Plugin)
    find_package(SofaPython3 REQUIRED COMPONENTS Plugin Bindings.Sofa)
endif()

SP3_add_python_module(
        TARGET       ${PROJECT_NAME}
        PACKAGE      Bindings.Sofa
        MODULE       SofaDiff
        DESTINATION  Sofa
        SOURCES      ${SOURCE_FILES}
        HEADERS      ${HEADER_FILES}
        DEPENDS      SofaDiff SofaPython3::Plugin SofaPython3::Bindings.Sofa
)

get_target_property(DIRS Bindings_SofaDiff INCLUDE_DIRECTORIES)
message(STATUS "Bindings_SofaDiff include dirs: ${DIRS}")

message("-- SofaPython3 bindings for SofaDiff will be created.")