add_executable(test test.cpp)
target_link_libraries(test linkbot)

add_executable(testGetters testGetters.cpp)
target_link_libraries(testGetters linkbot)

add_executable(testSetters testSetters.cpp)
target_link_libraries(testSetters linkbot)

add_executable(setJointMovementState setJointMovementState.cpp)
target_link_libraries(setJointMovementState linkbot)

add_executable(motorPower motorPower.cpp)
target_link_libraries(motorPower linkbot)

add_executable(testRecord testRecord.cpp)
target_link_libraries(testRecord linkbot)
