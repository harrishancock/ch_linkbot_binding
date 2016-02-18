include_directories(../src)

add_executable(testWrapper test.cpp)
target_link_libraries(testWrapper linkbot_wrapper)

add_executable(testGetters testGetters.cpp)
target_link_libraries(testGetters linkbot_wrapper)

add_executable(testSetters testSetters.cpp)
target_link_libraries(testSetters linkbot_wrapper)

add_executable(setJointMovementState setJointMovementState.cpp)
target_link_libraries(setJointMovementState linkbot_wrapper)

add_executable(motorPower motorPower.cpp)
target_link_libraries(motorPower linkbot_wrapper)

#add_executable(testRecord testRecord.cpp)
#target_link_libraries(testRecord linkbot)
