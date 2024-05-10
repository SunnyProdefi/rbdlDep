cmake_minimum_required(VERSION 3.0)
project(RBDL_Project)

# 确保使用C++11标准
set(CMAKE_CXX_STANDARD 11)
# 设置可执行文件的输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
# 查找RBDL库
find_package(RBDL REQUIRED)
# 如果Eigen安装在标准路径
include_directories("/usr/include/eigen3")

# 添加子目录
add_subdirectory(IK)
add_subdirectory(URDF)
add_subdirectory(SimpleExample)
add_subdirectory(IK_cs)
add_subdirectory(Kinematics)


