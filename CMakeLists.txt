# SPDX-License-Identifier: MIT

# --- cmake version ---
cmake_minimum_required(VERSION 3.21)

# --- project ---
project(ba7lya.abb.robot.egm VERSION 0.1.0 LANGUAGES CXX)

# --- build options ---
option(${PROJECT_NAME}_BUILD_BENCHMARK "Build benchmarks"    OFF)
option(${PROJECT_NAME}_BUILD_EXAMPLE   "Build examples"      OFF)
option(${PROJECT_NAME}_BUILD_DOC       "Build Documentation" OFF)
option(${PROJECT_NAME}_BUILD_TEST      "Build Test"          OFF)
option(BUILD_SHARED_LIBS               "Build dynamically-linked binaries" ON)

# --- includes ---
include(CTest)
include(GNUInstallDirs)
include(GoogleTest)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)

# --- dependencies ---
if(${PROJECT_NAME}_BUILD_BENCHMARK)
    find_package(benchmark CONFIG REQUIRED)
endif()
find_package(asio    CONFIG REQUIRED)
if(${PROJECT_NAME}_BUILD_TEST)
    find_package(GTest CONFIG REQUIRED)
endif()
find_package(Protobuf REQUIRED)
find_package(Threads  REQUIRED)

# Make sure protoc is present, as apparently the above find_package() doesn't check that.
if(NOT PROTOBUF_PROTOC_EXECUTABLE)
    message(FATAL_ERROR "Cannot find required 'protoc', cannot process Protobuf files without it. Aborting.")
endif()

# --- os ---
if(WIN32)
    add_definitions(-DWINVER=0x0A00)       # Windows 10
    add_definitions(-D_WIN32_WINNT=0x0A00) # Windows 10
    add_definitions(-DNOMINMAX)            # disable C2589 min()/max() error
endif()

# --- modules ---
add_subdirectory(thirdparty)

# --- protobuf ---
# Generate C++ for protocol classes (headers and sources get written to the CMAKE_CURRENT_BINARY_DIR location).
set(EgmProtoFiles proto/egm.proto proto/egm_wrapper.proto proto/egm_wrapper_trajectory.proto)
message(STATUS "Generating protobuf C++ for: ${EgmProtoFiles}")
if(MSVC)
    # Add export macro when using Microsoft Visual C++ compiler.
    protobuf_generate_cpp(EgmProtoSources EgmProtoHeaders EXPORT_MACRO ABB_EGM_EXPORT ${EgmProtoFiles})
else()
    protobuf_generate_cpp(EgmProtoSources EgmProtoHeaders ${EgmProtoFiles})
endif()

# --- target ---
set(
    PUBLIC_HEADERS
        include/base_interface.hxx
        include/common.hxx
        include/common_auxiliary.hxx
        include/controller_interface.hxx
        include/interpolator.hxx
        include/logger.hxx
        include/trajectory_interface.hxx
        include/udp_server.hxx
)

set(
    SRC_FILES
        src/base_interface.cxx
        src/common_auxiliary.cxx
        src/controller_interface.cxx
        src/interpolator.cxx
        src/logger.cxx
        src/trajectory_interface.cxx
        src/udp_server.cxx
        ${EgmProtoSources}
)

# Keep every runtime artifact of a configuration in one directory, so that executables built here find
# the shared library without any extra setup. There is no RPATH equivalent on Windows, and the loader
# searches the executable's own directory first, so sharing that directory with the library is what
# makes the tests, examples and benchmarks runnable straight from the build tree.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>")

add_library(${PROJECT_NAME} ${SRC_FILES})
add_library(ba7lya::abb::robot::egm ALIAS ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES EXPORT_NAME egm)

# The public headers include the generated protobuf headers as well as the export header by bare filename,
# so those are generated alongside them in the build directory and installed next to them (see below).
generate_export_header(${PROJECT_NAME} BASE_NAME abb_egm EXPORT_FILE_NAME export.hxx)

# Only the build directory is needed for the generated protobuf headers and the export header; protobuf's own
# include directory is inherited from the imported target below, so it never leaks into the installed package.
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    asio::asio
    protobuf::libprotobuf
    Threads::Threads
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)
target_compile_options(${PROJECT_NAME} PRIVATE "$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

if(NOT BUILD_SHARED_LIBS)
    target_compile_definitions(${PROJECT_NAME} PUBLIC "ABB_EGM_STATIC_DEFINE")
endif()

if(MSVC)
    # Force include the export header when using Microsoft Visual C++ compiler.
    target_compile_options(${PROJECT_NAME} PUBLIC "/FI${CMAKE_CURRENT_BINARY_DIR}/export.hxx")
    # The interface classes are not annotated with ABB_EGM_EXPORT, so a shared build would otherwise
    # export none of them. Exporting every symbol keeps the DLL usable by tests, examples and
    # benchmarks without annotating the whole public API.
    set_target_properties(${PROJECT_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# --- install ---
# The source tree keeps the public headers flat in include/ and they include one another by bare filename;
# a quoted include searches the including file's own directory first, so the headers themselves need no
# rewriting for the nested layout. They are installed to <prefix>/include/<author>/<package>/ because that
# is the only spelling that stays unambiguous once several ba7lya packages are installed into one prefix.
# The generated protobuf headers and the export header go to the same directory, as the public headers
# include them by bare filename as well.
install(
    FILES
        ${PUBLIC_HEADERS}
        ${EgmProtoFiles}
        ${EgmProtoHeaders}
        "${CMAKE_CURRENT_BINARY_DIR}/export.hxx"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ba7lya/abb/robot/egm
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Create the ${PROJECT_NAME}Config.cmake.
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY
)

# Create the ${PROJECT_NAME}ConfigVersion.cmake.
write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
        "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
)

# Export targets.
install(
    EXPORT export_${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ba7lya::abb::robot::
)

# --- benchmark ---
if(${PROJECT_NAME}_BUILD_BENCHMARK)
    add_subdirectory(benchmark)
endif()

# --- examples ---
if(${PROJECT_NAME}_BUILD_EXAMPLE)
    add_subdirectory(examples)
endif()

# --- unit tests ---
if(${PROJECT_NAME}_BUILD_TEST)
    enable_testing()
    add_subdirectory(tests)
endif()
