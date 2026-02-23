cmake_minimum_required(VERSION 3.20)
project(cgal_wrapper CXX)

# ── CMake policy compatibility ────────────────────────────────────────────────
# CMP0167 (cmake 3.30+): Use upstream BoostConfig.cmake over legacy FindBoost.
# CMP0169 (cmake 3.30+): Allow FetchContent_Populate() for header-only fetches
# where MakeAvailable would try to run the dependency's own CMakeLists.txt.
foreach(_pol CMP0167 CMP0169)
    if(POLICY ${_pol})
        cmake_policy(SET ${_pol} OLD)
    endif()
endforeach()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export all symbols from the DLL on Windows so that the extern "C" entry
# points are visible to P/Invoke without explicit __declspec(dllexport).
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

include(FetchContent)

# ── Fetch CGAL 6.1.1 header-only library ZIP (~10 MB) ────────────────────────
# The *-library.zip* is the header-only subset published on each CGAL release
# — far smaller than the full source archive.
FetchContent_Declare(
    CGAL_zip
    URL      https://github.com/CGAL/cgal/releases/download/v6.1.1/CGAL-6.1.1-library.zip
    URL_HASH SHA256=6c5d68be1d28cbee3c3e05003746ec4791d0018c770b4276b9e6d69c3a0a355a
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(CGAL_zip)
if(NOT cgal_zip_POPULATED)
    FetchContent_Populate(CGAL_zip)
endif()

# ── Fetch minimal Boost headers from individual boostorg GitHub repos ─────────
# CGAL 6.x needs several header-only Boost sub-libraries.  Instead of
# downloading the full Boost archive (>100 MB), we fetch only the repos that
# are actually required.  Each individual library ZIP is < 2 MB; total ≈ 20 MB.
# All headers are staged into a single directory so one include path covers all.
set(_BOOST_TAG "boost-1.87.0")

# Complete list of Boost sub-libraries required by CGAL 6.1.1 CDT (Epick):
set(_BOOST_LIBS
    algorithm any assert concept_check config container container_hash
    core describe exception foreach functional fusion integer intrusive io
    iterator lexical_cast math move mp11 mpl multiprecision
    numeric_conversion optional predef preprocessor property_map random
    range smart_ptr static_assert throw_exception tuple type_index
    type_traits utility
)

set(_BOOST_STAGING "${CMAKE_BINARY_DIR}/boost_headers")
file(MAKE_DIRECTORY "${_BOOST_STAGING}")

foreach(_lib IN LISTS _BOOST_LIBS)
    set(_fc "boost_${_lib}")
    FetchContent_Declare(
        ${_fc}
        URL      "https://github.com/boostorg/${_lib}/archive/refs/tags/${_BOOST_TAG}.zip"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_GetProperties(${_fc})
    if(NOT ${_fc}_POPULATED)
        FetchContent_Populate(${_fc})
    endif()
    # Each repo puts its headers under include/ — copy into the staging area
    if(EXISTS "${${_fc}_SOURCE_DIR}/include")
        file(COPY "${${_fc}_SOURCE_DIR}/include/"
             DESTINATION "${_BOOST_STAGING}")
    endif()
endforeach()

# ── Build the shared library ──────────────────────────────────────────────────
add_library(cgal_wrapper SHARED cgal_wrapper.cpp)

target_include_directories(cgal_wrapper PRIVATE
    "${cgal_zip_SOURCE_DIR}/include"
    "${_BOOST_STAGING}"
)

# Tell CGAL to skip GMP/MPFR: the Epick kernel (exact predicates, inexact
# constructions) uses only floating-point interval arithmetic which is
# entirely self-contained in the CGAL headers — no GMP/MPFR needed.
target_compile_definitions(cgal_wrapper PRIVATE CGAL_DISABLE_GMP=1)

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
