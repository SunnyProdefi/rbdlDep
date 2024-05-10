# 添加可执行文件
add_executable(IK_rbdl IK_rbdl.cpp)

# 链接 RBDL 库
target_link_libraries(IK_rbdl ${RBDL_LIBRARY})


