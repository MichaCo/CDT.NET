cmake_minimum_required(VERSION 3.20)
project(cgal_wrapper CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export all symbols from the DLL on Windows so that the extern "C" entry
# points are visible to P/Invoke without explicit __declspec(dllexport).
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# ── Locate CGAL ───────────────────────────────────────────────────────────────
# CGAL 5.x/6.x is header-only for 2D triangulation; only the header-only
# CGAL::CGAL target is needed.
#
# Install CGAL before building:
#   Linux  : sudo apt install libcgal-dev
#   macOS  : brew install cgal
#   Windows: winget install CGAL.CGAL   (or via vcpkg: vcpkg install cgal)
find_package(CGAL QUIET)
if(NOT CGAL_FOUND)
    message(FATAL_ERROR
        "CGAL was not found. Please install CGAL before building the cgal_wrapper:\n"
        "  Linux  : sudo apt install libcgal-dev\n"
        "  macOS  : brew install cgal\n"
        "  Windows: winget install CGAL.CGAL  (or via vcpkg: vcpkg install cgal)\n"
        "If you installed CGAL in a non-standard location, set CGAL_DIR to the "
        "directory containing CGALConfig.cmake.")
endif()

add_library(cgal_wrapper SHARED cgal_wrapper.cpp)
target_link_libraries(cgal_wrapper PRIVATE CGAL::CGAL)

set_target_properties(cgal_wrapper PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    # Always place the output directly in the build root directory.
    # Without this, multi-config generators (MSVC) put the DLL in
    # build/Release/ instead of build/, breaking the MSBuild copy step.
    RUNTIME_OUTPUT_DIRECTORY          "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE  "${CMAKE_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY          "${CMAKE_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE  "${CMAKE_BINARY_DIR}"
)
