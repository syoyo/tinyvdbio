# Assume this cmake is called from tinyvdbio root(../../)
set(EXAMPLE_TARGET "vdbdump")

set(TINYVDBIO_VDBDUMP_SOURCES
    main.cc
    )

add_executable(${EXAMPLE_TARGET} ${TINYVDBIO_VDBDUMP_SOURCES})
add_sanitizers(${EXAMPLE_TARGET})

target_include_directories(${EXAMPLE_TARGET} PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${EXAMPLE_TARGET} tinyvdbio)

set_target_properties(${EXAMPLE_TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
