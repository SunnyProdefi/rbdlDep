#include <iostream>
#include <vector>
#include <cmath>
#include <rbdl/rbdl.h> // 包含RBDL库头文件
#include <rbdl/Kinematics.h> // 包含RBDL运动学相关的头文件

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main() {
    Model model; // 创建一个模型实例

    // 定义连杆的质量、质心和惯性矩阵
    double mass = 1.0;
    Vector3d com(0.5, 0, 0);
    Matrix3d inertia = Matrix3d::Identity() * 0.1;

    // 创建关节，这里是一个绕Z轴旋转的关节
    Joint revoluteJoint(JointTypeRevolute, Vector3d(0, 0, 1)); // 创建一个关于Z轴的旋转关节

    // 添加第一个连杆到模型中
    Body body1(mass, com, inertia); // 为第一个连杆创建一个实体对象
    unsigned int body1_id = model.AddBody(0, SpatialTransform(), revoluteJoint, body1); // 将第一个连杆添加到模型中，返回连杆的ID

    // 添加第二个连杆
    Body body2(mass, com, inertia); // 为第二个连杆创建一个实体对象
    unsigned int body2_id = model.AddBody(body1_id, Xtrans(Vector3d(1.0, 0.0, 0.0)), revoluteJoint, body2); // 将第二个连杆附加到第一个连杆，返回连杆的ID

    // 添加第三个连杆
    Body body3(mass, com, inertia); // 为第三个连杆创建一个实体对象
    unsigned int body3_id = model.AddBody(body2_id, Xtrans(Vector3d(1.0, 0.0, 0.0)), revoluteJoint, body3); // 将第三个连杆附加到第二个连杆，返回连杆的ID

    // 初始化关节角度
    VectorNd Q = VectorNd::Zero(model.dof_count); // 创建一个向量来存储关节的角度，这里初始化为全零

    // 在第三个连杆的局部坐标系中定义一个点的位置
    Vector3d point_in_body_coords(1.0, 0.0, 0.0); // 假设连杆的长度为1米，我们取连杆末端的点

    // 计算并转换该点到基座标系（世界坐标系）中的位置
    Vector3d point_in_base_coords = CalcBodyToBaseCoordinates(model, Q, body3_id, point_in_body_coords, true);

    // 在控制台输出这个点在基座标系中的位置
    std::cout << "Point in base coordinates: " << point_in_base_coords.transpose() << std::endl;

    // 计算第三个连杆在世界坐标系中的方向
    Matrix3d body_world_orientation = CalcBodyWorldOrientation(model, Q, body3_id, true);

    // 在控制台输出第三个连杆的世界方向矩阵
    std::cout << "Body world orientation:\n" << body_world_orientation << std::endl;

    return 0;
}
