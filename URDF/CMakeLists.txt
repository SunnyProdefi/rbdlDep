add_executable(urdf_rbdl urdf_rbdl.cpp)
target_link_libraries(urdf_rbdl ${RBDL_LIBRARY} /usr/local/lib/librbdl_urdfreader.so)

