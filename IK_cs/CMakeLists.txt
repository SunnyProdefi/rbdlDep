# 添加可执行文件
add_executable(IK_cs IK_cs.cpp)

# 链接 RBDL 库
target_link_libraries(IK_cs ${RBDL_LIBRARY})


