# 添加可执行文件
add_executable(example example.cpp)

# 链接 RBDL 库
target_link_libraries(example ${RBDL_LIBRARY})


