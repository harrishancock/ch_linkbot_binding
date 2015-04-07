cmake_minimum_required(VERSION 2.8.12)
project(chlinkbot C CXX)

# Specify -DCH_ROOT=C:/path/to/ch on the command line, or set CH_ROOT as an
# environment variable, or we default to C:/Ch.
if(NOT DEFINED CH_ROOT)
	if (DEFINED ENV{CH_ROOT})
		set(CH_ROOT $ENV{CH_ROOT})
	else()
		set(CH_ROOT "C:/Ch")
	endif()
endif()
message(STATUS "Using Ch at ${CH_ROOT}")

# This CMakeLists.txt is a little weird, because it depends on a superdirectory,
# not a subdirectory. We should probably move ../CMakeLists.txt into ../src,
# and move ./CMakeLists.txt into ../.
add_subdirectory(.. __)

add_library(chlinkbot SHARED chlinkboti_chdl.cpp chlinkbotl_chdl.cpp)

# We want the library to be named liblinkbot.dl, not chlinkbot.dll.
if(WIN32)
set_target_properties(chlinkbot PROPERTIES OUTPUT_NAME liblinkbot SUFFIX .dl)
elseif(APPLE)
set_target_properties(chlinkbot PROPERTIES OUTPUT_NAME linkbot SUFFIX .dl)
set_target_properties(chlinkbot PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
else()
set_target_properties(chlinkbot PROPERTIES OUTPUT_NAME linkbot SUFFIX .dl)
endif()

target_include_directories(chlinkbot PUBLIC ${CH_ROOT}/extern/include)

# TODO these are Windows-specific paths. For a cross-platform CMake script, we
# should use find_library or at least find_file/find_path to get the library
# paths.
if(WIN32)
target_link_libraries(chlinkbot
	PUBLIC linkbot
		   ${CH_ROOT}/extern/lib/chsdk.lib		   
		   ${CH_ROOT}/extern/lib/ch.lib)
else()
target_link_libraries(chlinkbot
	PUBLIC linkbot
		   ${CH_ROOT}/extern/lib/libchsdk.a)
endif()

# TODO the install target of the old Makefile still needs to be converted into
# CMake.
#install:
#	sh ./createpkg.sh
#	ch pkginstall.ch chbarobo-0.2
