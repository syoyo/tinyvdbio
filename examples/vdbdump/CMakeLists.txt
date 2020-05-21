# Assume this cmake is called from tinyvdbio root(../../)
set(EXAMPLE_TARGET "vdbdump")

set(TINYVDBIO_VDBDUMP_SOURCES
    vdbdump.cc
    )

add_executable(${EXAMPLE_TARGET} ${TINYVDBIO_VDBDUMP_SOURCES})
add_sanitizers(${EXAMPLE_TARGET})

target_include_directories(${EXAMPLE_TARGET} PRIVATE ${PROJECT_SOURCE_DIR}/src)

# for clipp.h
target_include_directories(${EXAMPLE_TARGET} PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)

target_link_libraries(${EXAMPLE_TARGET} tinyvdbio)

if (TINYVDBIO_USE_SYSTEM_ZLIB)
  target_compile_definitions(${EXAMPLE_TARGET} PRIVATE "TINYVDBIO_USE_SYSTEM_ZLIB")
  target_link_libraries(${EXAMPLE_TARGET} PUBLIC ZLIB::ZLIB)
endif (TINYVDBIO_USE_SYSTEM_ZLIB)

set_target_properties(${EXAMPLE_TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
