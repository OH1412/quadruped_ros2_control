cmake_minimum_required(VERSION 3.8)
project(unitree_hw_interface)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(unitree_motor_msgs REQUIRED)

# ==========================================
# SDK 路径修复：优先使用环境变量
# ==========================================
if(DEFINED ENV{UNITREE_ACTUATOR_SDK_ROOT})
    set(UNITREE_SDK_ROOT $ENV{UNITREE_ACTUATOR_SDK_ROOT})
else()
    # 如果没搜到环境变量，回退到你原本的相对路径尝试（已修正层级）
    set(UNITREE_SDK_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../../..) 
endif()

message(STATUS "Using Unitree SDK Root: ${UNITREE_SDK_ROOT}")

# 头文件路径包含
set(UNITREE_SDK_INCLUDE 
    ${UNITREE_SDK_ROOT}/include
    ${UNITREE_SDK_ROOT}/thirdparty/serialPort # 解决 SerialPort.h 找不到的问题
)
set(UNITREE_SDK_LIB_DIR ${UNITREE_SDK_ROOT}/lib)

# 库文件名检查
# 注意：宇树官方库有时叫 libunitree_actuator_sdk.so，有时叫 libUnitreeMotorSDK_Linux64.so
# 我们优先寻找你之前编译生成的那个
find_library(UNITREE_SDK_LIB
  NAMES unitree_actuator_sdk UnitreeMotorSDK_Linux64 UnitreeMotorSDK_Arm64
  PATHS ${UNITREE_SDK_LIB_DIR}
  NO_DEFAULT_PATH
)

add_library(${PROJECT_NAME} SHARED
  src/unitree_hw_interface.cpp
)

pluginlib_export_plugin_description_file(hardware_interface unitree_hw_interface.xml)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  unitree_motor_msgs
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${UNITREE_SDK_INCLUDE}
)

target_link_libraries(${PROJECT_NAME}
  ${UNITREE_SDK_LIB}
)

# ==========================================

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES unitree_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()