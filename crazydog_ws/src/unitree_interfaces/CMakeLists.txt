cmake_minimum_required(VERSION 3.8)
project(unitree_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_msgs REQUIRED)

set(UNITREE_SDK_PATH "/home/crazydog/crazydog/crazydog_ws/src/unitree_interfaces/unitree_actuator_sdk")  # Change this to the path where the SDK is located

# Include directories
include_directories(${UNITREE_SDK_PATH}/include)
# include_directories(${UNITREE_SDK_PATH}/include/serialPort/include)
# include_directories(${UNITREE_SDK_PATH}/include/unitreeMotor/include)
link_directories(${UNITREE_SDK_PATH}/lib)

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64") 
    set(EXTRA_LIBS ${UNITREE_SDK_PATH}/lib/libUnitreeMotorSDK_Arm64.so)
else()
    set(EXTRA_LIBS ${UNITREE_SDK_PATH}/lib/libUnitreeMotorSDK_Linux64.so)
endif()

# Add the Unitree interfaces executable
add_executable(unitree_pubsub src/unitree_pubsub.cpp)
add_executable(unitree_test src/test.cpp)
ament_target_dependencies(unitree_pubsub rclcpp std_msgs sensor_msgs unitree_msgs)
ament_target_dependencies(unitree_test rclcpp std_msgs sensor_msgs unitree_msgs)

target_link_libraries(unitree_pubsub ${EXTRA_LIBS})  # Link with the Unitree SDK
target_link_libraries(unitree_test ${EXTRA_LIBS})  # Link with the Unitree SDK


# Install the executable
install(TARGETS
  unitree_pubsub
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  unitree_test
  DESTINATION lib/${PROJECT_NAME}
)


# Install include directories
# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME}/
# )

ament_package()

