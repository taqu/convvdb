cmake_minimum_required(VERSION 3.29)

set(CMAKE_CONFIGURATION_TYPES "Debug" "Release")

set(PROJECT_NAME convvdb)
project(${PROJECT_NAME} CXX)

set(PROJECT_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(AFTER ${PROJECT_ROOT}/include)
set(HEADER_DIR ${PROJECT_ROOT}/include)
set(SOURCE_DIR ${PROJECT_ROOT}/src)
set(THIRDPARTY_ROOT ${PROJECT_ROOT}/thirdparty)

#######################################################
# TBB
include_directories(AFTER ${THIRDPARTY_ROOT}/tbb)

#######################################################
# OpenVDB
set(OPENVDB_ROOT ${THIRDPARTY_ROOT}/openvdb)
set(OPENVDB_INCLUDE ${OPENVDB_ROOT}/include)
set(OPENVDB_LIB ${OPENVDB_ROOT}/lib)

include_directories(AFTER ${OPENVDB_INCLUDE})
link_directories(AFTER ${OPENVDB_LIB})

add_library(OPENVDB MODULE IMPORTED)
if(MSVC)
    set_target_properties(OPENVDB PROPERTIES IMPORTED_LOCATION_DEBUG "openvdbd.lib" IMPORTED_LOCATION_RELEASE "openvdb.lib")
else()
    set_target_properties(OPENVDB PROPERTIES IMPORTED_LOCATION "openvdbd.a" IMPORTED_LOCATION_RELEASE "openvdb.a")
endif()

set(HEADERS
    ${HEADER_DIR}/args.hxx
    ${HEADER_DIR}/cppimg.h)

set(SOURCES
    ${SOURCE_DIR}/cppimg.cpp
    ${SOURCE_DIR}/convvdb.cpp)

source_group("include" FILES ${HEADERS})
source_group("src" FILES ${SOURCES})

set(FILES ${HEADERS} ${SOURCES})

set(OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}")

add_executable(${PROJECT_NAME} ${FILES})
if(MSVC)
    set(DEFAULT_CXX_FLAGS "/DWIN32 /D_WINDOWS /D_MBCS /DCPPIMG_DISABLE_PNG /DCPPIMG_DISABLE_OPENEXR /DOPENVDB_DLL /DOPENVDB_USE_DELAYED_LOADING /W4 /WX- /nologo /fp:precise /arch:AVX2 /Zc:wchar_t /TP /Gd /utf-8")
    if(MSVC_VERSION VERSION_LESS_EQUAL "1900")
        set(DEFAULT_CXX_FLAGS "${DEFAULT_CXX_FLAGS} /Zc:__cplusplus")
    else()
        set(DEFAULT_CXX_FLAGS "${DEFAULT_CXX_FLAGS} /Zc:__cplusplus")
    endif()
    set(CMAKE_CXX_FLAGS "${DEFAULT_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS_DEBUG "/D_DEBUG /MTd /Zi /Ob0 /Od /RTC1 /Gy /GR /GS /Gm- /EHsc")
    set(CMAKE_CXX_FLAGS_RELEASE "/MT /O2 /Oi /GL /GR /DNDEBUG /EHsc-")
    target_link_libraries(${PROJECT_NAME} OPENVDB)

elseif(UNIX)
    set(DEFAULT_CXX_FLAGS "-Wall -O2 -std=c++20 -std=gnu++20 -march=x86-64-v3 -mavx2 -fno-exceptions -DCPPIMG_DISABLE_PNG -DCPPIMG_DISABLE_OPENEXR -DOPENVDB_DLL -DOPENVDB_USE_DELAYED_LOADING")
    set(CMAKE_CXX_FLAGS "${DEFAULT_CXX_FLAGS}")
    target_link_libraries(${PROJECT_NAME} OPENVDB)
elseif(APPLE)
endif()

#set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_ARCHITECTURES "80;90")
#set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 23)
set_target_properties(${PROJECT_NAME}
    PROPERTIES
        OUTPUT_NAME_DEBUG "${PROJECT_NAME}" OUTPUT_NAME_RELEASE "${PROJECT_NAME}"
        VS_DEBUGGER_COMMAND_ARGUMENTS "SmallCampfireVDB/smallCampfire_0000.vdb --output=output"
        VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

