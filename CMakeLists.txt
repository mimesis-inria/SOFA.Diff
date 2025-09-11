cmake_minimum_required(VERSION 3.22)

#replace here with your project name and version number
project(SofaDiff VERSION 0.1)

#look for all packages on which your project depends
find_package(Sofa.Framework REQUIRED)
find_package(Sofa.Component REQUIRED)

set(HEADER_FILES
        src/SofaDiff/initSofaDiff.h
        src/SofaDiff/GradientDescentController.h
)

set(SOURCE_FILES
        src/SofaDiff/initSofaDiff.cpp
        src/SofaDiff/GradientDescentController.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} Sofa.Core)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSOFA_BUILD_SOFADIFF")
