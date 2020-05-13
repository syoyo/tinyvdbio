cmake_minimum_required(VERSION 3.5)

set(BUILD_TARGET "tinyvdbio")
set(TEST_TARGET "test_tinyvdbio")

project(${BUILD_TARGET} CXX C)

# options
option(TINYVDBIO_USE_CCACHE "Use ccache for faster recompile." ON)
option(TINYVDBIO_BUILD_TESTS "Build tests" ON)
option(TINYVDBIO_BUILD_EXAMPLES "Build examples(but not all examples area built in `examples` folder)" ON)
option(TINYVDBIO_USE_BLOSC "Build with blosc support(Usually recent(2019~) .vdb files are compressed by BLOSC)" ON)
option(TINYVDBIO_USE_SYSTEM_ZLIB "Do not use miniz.c when ON)" OFF)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/sanitizers)
find_package(Sanitizers) # Address sanitizer (-DSANITIZE_ADDRESS=ON)

# Require strict C++11 mode(e.g. `-std=c++11`)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# [ccache]
if(TINYVDBIO_USE_CCACHE)
  if(MSVC)
    # No ccache support
  else()
    find_program(CCACHE_EXE ccache)
    if(CCACHE_EXE)

      message(STATUS "Use ccache : " ${CCACHE_EXE})
      # CMAKE_C_COMPILER_LAUNCHER = available from cmake 3.4+)
      if(CMAKE_C_COMPILER_LAUNCHER)
        set(CMAKE_C_COMPILER_LAUNCHER "${CMAKE_C_COMPILER_LAUNCHER}"
                                      "${CCACHE_EXE}")
      else()
        set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_EXE}")
      endif()

      if(CMAKE_CXX_COMPILER_LAUNCHER)
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CMAKE_CXX_COMPILER_LAUNCHER}"
                                        "${CCACHE_EXE}")
      else()
        set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_EXE}")
      endif()

    endif(CCACHE_EXE)
  endif()
endif()

set(TINYVDBIO_SOURCES
    ${PROJECT_SOURCE_DIR}/src/tinyvdbio.cc
    )

set(TINYVDBIO_DEP_SOURCES
  )

# Force use system zlib for IOS
if (IOS)
  set(TINYVDBIO_USE_SYSTEM_ZLIB ON CACHE INTERNAL "" FORCE)
endif (IOS)

if (NOT TINYVDBIO_USE_SYSTEM_ZLIB)
  list(APPEND TINYVDBIO_DEP_SOURCES
    ${PROJECT_SOURCE_DIR}/src/miniz.c
  )

  list(APPEND TINYVDBIO_EXT_LIBRARIES blosc_static)
endif (NOT TINYVDBIO_USE_SYSTEM_ZLIB)

if (TINYVDBIO_USE_BLOSC)

  # Use cmake FUNCTION feature to define cmake variables only effective for c-blosc project
  # https://stackoverflow.com/questions/41700463/push-pop-a-cmake-variable
  function ( add_blosc_lib )
    set(BUILD_TESTS OFF CACHE INTERNAL "" FORCE)
    set(BUILD_BENCHMARKS OFF CACHE INTERNAL "" FORCE)

    # TODO(syoyo): Only disable SSE2 for non-x86 platforms
    set(DEACTIVATE_SSE2 OFF CACHE INTERNAL "" FORCE)
    set(DEACTIVATE_AVX2 OFF CACHE INTERNAL "" FORCE)

    # TODO(syoyo): Enable shared blosc.so build
    set(BUILD_SHARED OFF CACHE INTERNAL "" FORCE)

    # It looks OpenVDB does not use zstd so disable it
    set(DEACTIVATE_ZSTD ON CACHE INTERNAL "" FORCE)

    if (IOS)
      set(DEACTIVATE_ZLIB ON CACHE INTERNAL "" FORCE)
    endif (IOS)

    add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/c-blosc)
  endfunction ()

  add_blosc_lib()

endif (TINYVDBIO_USE_BLOSC)

add_library(${BUILD_TARGET} ${TINYVDBIO_SOURCES} ${TINYVDBIO_DEP_SOURCES})
add_sanitizers(${BUILD_TARGET})

target_include_directories(${BUILD_TARGET} PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(${BUILD_TARGET} ${TINYVDBIO_EXT_LIBRARIES} ${CMAKE_DL_LIBS})

if (TINYVDBIO_USE_SYSTEM_ZLIB)
  target_compile_definitions(${BUILD_TARGET} PRIVATE "TINYVDBIO_USE_SYSTEM_ZLIB")
endif (TINYVDBIO_USE_SYSTEM_ZLIB)

if (TINYVDBIO_USE_BLOSC)
  target_include_directories(${BUILD_TARGET} PRIVATE ${PROJECT_SOURCE_DIR}/third_party/c-blosc/blosc)
  target_compile_definitions(${BUILD_TARGET} PRIVATE "TINYVDBIO_USE_BLOSC")
endif (TINYVDBIO_USE_BLOSC)

# Increase warning level for clang.
IF (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set_source_files_properties(${TINYVDBIO_SOURCES} PROPERTIES COMPILE_FLAGS "-Weverything -Werror -Wno-padded -Wno-c++98-compat-pedantic -Wno-documentation -Wno-unused-member-function")
ENDIF ()

if (TINYVDBIO_BUILD_EXAMPLES)
  add_subdirectory(examples/vdbdump)
endif (TINYVDBIO_BUILD_EXAMPLES)

if (TINYVDBIO_BUILD_TESTS)

  set(TINYVDBIO_TEST_SOURCES
      ${PROJECT_SOURCE_DIR}/tests/loader-test.cc
      )

  add_executable(${TEST_TARGET} ${TINYVDBIO_TEST_SOURCES})
  add_sanitizers(${TEST_TARGET})

  target_include_directories(${TEST_TARGET} PRIVATE ${PROJECT_SOURCE_DIR}/src
                                                    ${PROJECT_SOURCE_DIR}/tests)
  target_link_libraries(${TEST_TARGET} PRIVATE ${BUILD_TARGET})

endif (TINYVDBIO_BUILD_TESTS)

# [VisualStudio]
if(WIN32)
  # Set ${BUILD_TARGET} as a startup project for VS IDE
  set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${BUILD_TARGET})

  # For easier debugging in VS IDE(cmake 3.8.0 or later required) Set working
  # directory to ${BUILD_TARGET} git repo root.
  if(CMAKE_VERSION VERSION_GREATER 3.8.0)
    set_target_properties(${BUILD_TARGET}
                          PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY
                                     "${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
endif()
