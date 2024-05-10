#include <iostream>
#include <vector>
#include <cmath>
#include <rbdl/rbdl.h> // 引入RBDL库
#include <rbdl/Kinematics.h> // 引入RBDL运动学相关功能

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main() {
    Model model; // 创建模型对象

    // 定义连杆的质量、质心和惯性矩阵
    double mass = 1.0; // 连杆的质量
    Vector3d com(0.5, 0, 0); // 连杆的质心
    Matrix3d inertia = Matrix3d::Identity() * 0.1; // 连杆的惯性矩阵，使用单位矩阵乘以0.1作为示例

    // 创建三个连杆，每个连杆都绕Z轴旋转
    Joint revoluteJoint(JointTypeRevolute, Vector3d(0, 0, 1)); // 绕Z轴旋转的关节定义

    // 添加第一个连杆
    Body body1(mass, com, inertia);
    unsigned int body1_id = model.AddBody(0, SpatialTransform(), revoluteJoint, body1);

    // 添加第二个连杆
    Body body2(mass, com, inertia);
    unsigned int body2_id = model.AddBody(body1_id, Xtrans(Vector3d(1.0, 0.0, 0.0)), revoluteJoint, body2);

    // 添加第三个连杆
    Body body3(mass, com, inertia);
    unsigned int body3_id = model.AddBody(body2_id, Xtrans(Vector3d(1.0, 0.0, 0.0)), revoluteJoint, body3);

    // 准备逆向运动学约束集合
    InverseKinematicsConstraintSet ik_constraints;
    ik_constraints.AddPointConstraint(body3_id, Vector3d(1.0, 0.0, 0.0), Vector3d(2.5, 0.5, 0.0));

    ik_constraints.lambda = 0.01; // 逆运动学求解器的阻尼系数

    // 初始猜测和关节配置结果容器
    VectorNd Qinit = VectorNd::Constant(model.dof_count, 0.0); // 将所有初始猜测设置为0
    VectorNd Qres = VectorNd::Constant(model.dof_count, 0.0); // 逆运动学的结果

    // 执行逆向运动学计算
    bool success = InverseKinematics(model, Qinit, ik_constraints, Qres);

    // 输出结果
    if (success) {
        std::cout << "IK solution found (°): ";
        for (size_t i = 0; i < model.dof_count; ++i) {
            std::cout << Qres[i] * (180.0 / M_PI) << " "; // 转换为度数以便于阅读
        }
        std::cout << std::endl;
    } else {
        std::cout << "Failed to find IK solution." << std::endl;
    }

    return 0;
}
