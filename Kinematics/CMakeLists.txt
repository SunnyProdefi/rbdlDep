# 添加可执行文件
add_executable(kine kine.cpp)

# 链接 RBDL 库
target_link_libraries(kine ${RBDL_LIBRARY})
