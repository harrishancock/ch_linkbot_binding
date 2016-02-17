cmake_minimum_required(VERSION 3.2.1) 
project(liblinkbot VERSION 1.0.9 LANGUAGES C CXX)

if(CMAKE_CROSSCOMPILING)
    find_library(PTHREAD pthread ${CMAKE_PREFIX_PATH} REQUIRED)
    #set(Boost_COMPILER -gcc)
    #add_compile_options(${CMAKE_CROSSCOMPILE_C_FLAGS})
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_CROSSCOMPILE_C_FLAGS} -static-libstdc++ -static-libgcc")
endif(CMAKE_CROSSCOMPILING)

add_subdirectory(sdk)
add_subdirectory(src)
add_subdirectory(test)

install(DIRECTORY chbarobo DESTINATION . PATTERN ".gitkeep" EXCLUDE)
if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        install(DIRECTORY dlls/Microsoft.VC80.CRT DESTINATION chbarobo/dl)
    endif()
endif()

set(CPACK_PACKAGE_NAME "Linkbot Ch Package")
set(CPACK_PACKAGE_VENDOR "Barobo, Inc.")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})

# The upgrade GUID should remain constant for the lifetime of this codebase.
set(CPACK_WIX_UPGRADE_GUID "0c082802-8077-4d4b-b372-799f50ed1fcb")
set(CPACK_WIX_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/WIX.template.in")
set(CPACK_WIX_PATCH_FILE "${CMAKE_CURRENT_SOURCE_DIR}/WIX.patches")

include(CPack)