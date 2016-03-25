cmake_minimum_required(VERSION 3.2.1)
project(liblinkbot VERSION 1.0.11 LANGUAGES C CXX)

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
    file(TO_CMAKE_PATH "$ENV{ProgramFiles\(x86\)}/Microsoft Visual Studio 12.0/VC/redist"
                       vcredistDir)
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        # The 32-bit version of the Ch SDK seems to depend on MSVC 8.0's runtime
        install(DIRECTORY dlls/Microsoft.VC80.CRT DESTINATION chbarobo/dl)
        install(DIRECTORY "${vcredistDir}/x86/Microsoft.VC120.CRT/"
                DESTINATION chbarobo/bin)
    else()
        install(DIRECTORY "${vcredistDir}/x64/Microsoft.VC120.CRT/"
                DESTINATION chbarobo/bin)
    endif()
endif()
