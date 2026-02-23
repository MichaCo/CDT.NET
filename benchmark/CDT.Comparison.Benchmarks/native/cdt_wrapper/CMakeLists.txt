cmake_minimum_required(VERSION 3.20)
project(cdt_wrapper CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export all symbols from the DLL on Windows so that the extern "C" entry
# points are visible to P/Invoke without explicit __declspec(dllexport).
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

include(FetchContent)
FetchContent_Declare(
    CDT_Upstream
    GIT_REPOSITORY https://github.com/artem-ogre/CDT.git
    GIT_TAG        1.4.4
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  CDT
)
FetchContent_MakeAvailable(CDT_Upstream)

add_library(cdt_wrapper SHARED cdt_wrapper.cpp)
target_link_libraries(cdt_wrapper PRIVATE CDT)
set_target_properties(cdt_wrapper PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    # Always place the output directly in the build root directory.
    # Without this, multi-config generators (MSVC) put the DLL in
    # build/Release/ instead of build/, breaking the MSBuild copy step.
    RUNTIME_OUTPUT_DIRECTORY          "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE  "${CMAKE_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY          "${CMAKE_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE  "${CMAKE_BINARY_DIR}"
)
