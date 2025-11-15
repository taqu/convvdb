cmake_minimum_required(VERSION 3.50)

function(copy_dlls TARGET_NAME)
    add_custom_command (
        TARGET "${TARGET_NAME}" POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E copy -t "$<TARGET_FILE_DIR:${TARGET_NAME}>"
                "$<TARGET_RUNTIME_DLLS:${TARGET_NAME}>" USES_TERMINAL COMMAND_EXPAND_LISTS
    )
endfunction()

set(CMAKE_CONFIGURATION_TYPES "Debug" "Release")

set(PROJECT_NAME testvbd)
project(${PROJECT_NAME})

set(HEADER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(THIRDPARTY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty")

include_directories(AFTER "${HEADER_ROOT}/")
include_directories(AFTER "${THIRDPARTY_ROOT}/")

#######################################################
# OpenVDB
set(OPENVDB_ROOT ${THIRDPARTY_ROOT}/openvdb)
set(OPENVDB_INCLUDE ${OPENVDB_ROOT}/include)
set(OPENVDB_LIB ${OPENVDB_ROOT}/lib)

include_directories(AFTER ${OPENVDB_INCLUDE})
link_directories(AFTER ${OPENVDB_LIB})

add_library(OPENVDB MODULE IMPORTED)
if(MSVC)
    set_target_properties(OPENVDB PROPERTIES IMPORTED_LOCATION_DEBUG "libopenvdbd.lib" IMPORTED_LOCATION_RELEASE "libopenvdb.lib")
else()
    set_target_properties(OPENVDB PROPERTIES IMPORTED_LOCATION "libopenvdbd.a" IMPORTED_LOCATION_RELEASE "libopenvdb.a")
endif()

# TBB
find_package(TBB ${MINIMUM_TBB_VERSION} REQUIRED COMPONENTS tbb)
if(OPENVDB_FUTURE_DEPRECATION AND FUTURE_MINIMUM_TBB_VERSION)
  if(${TBB_VERSION} VERSION_LESS FUTURE_MINIMUM_TBB_VERSION)
    message(DEPRECATION "Support for TBB versions < ${FUTURE_MINIMUM_TBB_VERSION} "
      "is deprecated and will be removed.")
  endif()
endif()

########################################################################
# Sources
set(HEADERS
    "${HEADER_ROOT}/args.hxx"
    "${HEADER_ROOT}/cppimg.h"
    "${HEADER_ROOT}/testvbd.h")

set(SOURCES
    "${SOURCE_ROOT}/cppimg.cpp"
    "${SOURCE_ROOT}/testvbd.cpp")

source_group("include" FILES ${HEADERS})
source_group("src" FILES ${SOURCES})

set(FILES ${HEADERS} ${SOURCES})

set(OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}")

add_executable(${PROJECT_NAME} ${FILES})

if(MSVC)
    set(DEFAULT_CXX_FLAGS "/DWIN32 /D_WINDOWS /D_MSBC /DOPENVDB_STATICLIB /DCPPIMG_DISABLE_PNG /DCPPIMG_DISABLE_OPENEXR /W4 /WX- /nologo /fp:precise /arch:AVX2 /Zc:wchar_t /TP /Gd")
    if("1800" VERSION_LESS MSVC_VERSION)
        set(DEFAULT_CXX_FLAGS "${DEFAULT_CXX_FLAGS} /EHsc")
    endif()

    set(CMAKE_CXX_FLAGS "${DEFAULT_CXX_FLAGS}")
    set(CMAKE_CXX_FLAGS_DEBUG "/D_DEBUG /MDd /Zi /Ob0 /Od /RTC1 /Gy /GR- /GS /Gm-")
    set(CMAKE_CXX_FLAGS_RELEASE "/MD /O2 /GL /GR- /DNDEBUG")
    target_link_libraries(${PROJECT_NAME} OPENVDB TBB::tbb)

elseif(UNIX)
    set(DEFAULT_CXX_FLAGS "-Wall -O0 -g -DCPPHTTPLIB_OPENSSL_SUPPORT -DCPPIMG_DISABLE_PNG -DCPPIMG_DISABLE_OPENEXR  -DCPPHTTPLIB_ZLIB_SUPPORT -std=c++20 -std=gnu++20 -march=native")
    set(CMAKE_CXX_FLAGS "${DEFAULT_CXX_FLAGS}")
    target_link_libraries(${PROJECT_NAME} OPENVDB TBB::tbb)
elseif(APPLE)
endif()

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 23)
set_target_properties(${PROJECT_NAME}
    PROPERTIES
        OUTPUT_NAME_DEBUG "${PROJECT_NAME}" OUTPUT_NAME_RELEASE "${PROJECT_NAME}"
        VS_DEBUGGER_COMMAND_ARGUMENTS "../AerialExplosionVDB/AerialExplosion_0000.vdb"
        VS_DEBUGGER_WORKING_DIRECTORY "${OUTPUT_DIRECTORY}")
copy_dlls(${PROJECT_NAME})
