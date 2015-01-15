cmake_minimum_required(VERSION 2.8.12) 
project(liblinkbot CXX)

# Give subprojects access to LinkbotLabs.cmake
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/../cmake-modules")

include(LinkbotLabs)
ll_add_dependency(baromesh/library baromesh)

if(CMAKE_CROSSCOMPILING)
    find_library(PTHREAD pthread ${CMAKE_PREFIX_PATH} REQUIRED)
    #set(Boost_COMPILER -gcc)
    #add_compile_options(${CMAKE_CROSSCOMPILE_C_FLAGS})
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_CROSSCOMPILE_C_FLAGS} -static-libstdc++ -static-libgcc")
endif(CMAKE_CROSSCOMPILING)

#add_subdirectory(../deps/cxx-util cxx-util)
#add_subdirectory(../deps/libsfp libsfp)
#add_subdirectory(../deps/ribbon-bridge ribbon-bridge)
#add_subdirectory(../deps/ribbon-bridge-interfaces ribbon-bridge-interfaces)
#add_subdirectory(../deps/baromesh baromesh)

##############################################################################
# Boost

# Linking statically to Boost is easier for prototyping. For multi-module
# projects we'll want to use shared libraries and defined -DBOOST_ALL_DYN_LINK
# or the like.
set(Boost_USE_STATIC_LIBS ON)
find_package(Boost 1.54.0 REQUIRED COMPONENTS system log filesystem thread)
find_package(Threads)

set(SOURCES
    src/linkbot_wrapper.cpp 
    src/rgbhashtable.c.cpp
    )

add_library(linkbot ${SOURCES})

set_source_files_properties(${SOURCES}
    PROPERTIES COMPILE_FLAGS "-std=c++11 -Wno-write-strings")

target_include_directories(linkbot
    PUBLIC $<TARGET_PROPERTY:sfp,INCLUDE_DIRECTORIES>
           $<TARGET_PROPERTY:robot-interface,INCLUDE_DIRECTORIES>
           $<TARGET_PROPERTY:baromesh,INCLUDE_DIRECTORIES>
           $<TARGET_PROPERTY:cxx-util,INTERFACE_INCLUDE_DIRECTORIES>
           ${Boost_INCLUDE_DIRS}
           include
    )
target_link_libraries(linkbot
    PUBLIC sfp cxx-util
           robot-interface dongle-interface
           baromesh
           rpc 
           pthread
           ${Boost_LIBRARIES}
    )

if(WIN32)
    target_link_libraries(linkbot
        PRIVATE setupapi
        )
endif()

#add_subdirectory(test)
