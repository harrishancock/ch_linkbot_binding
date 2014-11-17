add_executable(test test.cpp)
target_link_libraries(test linkbot)

add_executable(testGetters testGetters.cpp)
target_link_libraries(testGetters linkbot)

add_executable(testSetters testSetters.cpp)
target_link_libraries(testSetters linkbot)
