cmake_minimum_required(VERSION 3.20)
project(mcl_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(MCL_SOURCE ${REPO_ROOT}/src/mcl.cpp CACHE FILEPATH "mcl.cpp to build against")

set(PROJECT_HEADERS mcl.h fieldRepresentation.h particle.h utilities.hpp telemetry.h)
set(HEADER_COPY_DIR ${CMAKE_CURRENT_BINARY_DIR}/project_include)
foreach(header ${PROJECT_HEADERS})
    configure_file(${REPO_ROOT}/include/${header} ${HEADER_COPY_DIR}/${header} COPYONLY)
endforeach()

add_executable(mcl_sim
    sim_main.cpp
    ${MCL_SOURCE}
    ${REPO_ROOT}/src/fieldRepresentation.cpp
    ${REPO_ROOT}/src/telemetry.cpp
)
target_include_directories(mcl_sim PRIVATE ${HEADER_COPY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/stubs)
if(MSVC)
    target_compile_options(mcl_sim PRIVATE /utf-8 /W3)
    target_compile_definitions(mcl_sim PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
