cmake_minimum_required(VERSION 3.20)
project(cdt_wrapper CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    CDT_Upstream
    GIT_REPOSITORY https://github.com/artem-ogre/CDT.git
    GIT_TAG        master
    GIT_SHALLOW    TRUE
    SOURCE_SUBDIR  CDT
)
FetchContent_MakeAvailable(CDT_Upstream)

add_library(cdt_wrapper SHARED cdt_wrapper.cpp)
target_link_libraries(cdt_wrapper PRIVATE CDT)
set_target_properties(cdt_wrapper PROPERTIES POSITION_INDEPENDENT_CODE ON)
