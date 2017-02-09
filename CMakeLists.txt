cmake_minimum_required(VERSION 3.2.1) 
project(liblinkbot VERSION 1.0.11 LANGUAGES C CXX)

if(CMAKE_CROSSCOMPILING)
    find_library(PTHREAD pthread ${CMAKE_PREFIX_PATH} REQUIRED)
    #set(Boost_COMPILER -gcc)
    #add_compile_options(${CMAKE_CROSSCOMPILE_C_FLAGS})
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_CROSSCOMPILE_C_FLAGS} -static-libstdc++ -static-libgcc")
endif(CMAKE_CROSSCOMPILING)

if(MSVC)
  # Don't warn on using strncpy(), fopen() instead of Microsoft's _s versions.
  add_compile_options(/W4
    /DWINVER=0x0600
    /D_WIN32_WINNT=0x0600
    /D_CRT_SECURE_NO_WARNINGS # Don't warn on strncpy() vs strncpy_s(), etc.
    /wd4714 # __forceinline function not inlined, happens in Boost.Log
    /wd4512 # assignment operator could not be generated, happens in Qt
    )
endif()

add_subdirectory(sdk)
add_subdirectory(src)
add_subdirectory(test)

install(DIRECTORY chbarobo DESTINATION . PATTERN ".gitkeep" EXCLUDE)
if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        install(DIRECTORY dlls/win32/ DESTINATION chbarobo/bin)
        install(DIRECTORY dlls/Microsoft.VC80.CRT DESTINATION chbarobo/dl)
    else()
        install(DIRECTORY dlls/win64/ DESTINATION chbarobo/bin)
    endif()
endif()
