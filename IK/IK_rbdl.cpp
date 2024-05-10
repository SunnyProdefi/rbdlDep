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
    // 定义末端执行器的目标位置
    std::vector<unsigned int> body_ids = { body3_id }; // 末端执行器的ID
    std::vector<Vector3d> target_positions = { Vector3d(1.0, 0.0, 0.0) }; // 末端执行器的目标位置
    std::vector<Vector3d> target_orientations; // 需要目标方向向量
    target_orientations.push_back(Vector3d(2.5, 0.5, 0.0)); // 示例目标方向
    // 初始化关节角度向量
    VectorNd Q = VectorNd::Zero(model.q_size); // 创建一个初始为0的关节角度向量
    // 设置算法参数
    double tolerance = 1e-6; // 容忍度
    double step_size = 1e-2; // 步长
    unsigned int max_iter = 1000; // 最大迭代次数
    // 执行逆运动学计算
    bool result = InverseKinematics(
        model, 
        Q, 
        body_ids, 
        target_positions, 
        target_orientations,
        Q, 
        tolerance, 
        step_size, 
        max_iter
    );
    // 输出结果
    if (result) {
        std::cout << "IK solution found: " << Q.transpose() << std::endl; // 如果找到解决方案，输出解决方案
    } else {
        std::cout << "Failed to find IK solution." << std::endl; // 如果未找到解决方案，输出失败信息
    }
    // 输出结果
    if (result) {
        // 输出找到的解决方案，并将弧度转换为度
        std::cout << "IK solution found in degrees:";
        for (int i = 0; i < Q.size(); ++i) {
            std::cout << " " << Q[i] * (180.0 / M_PI); // Convert each element from radians to degrees
        }
        std::cout << std::endl;
    } else {
        std::cout << "Failed to find IK solution." << std::endl; // 如果未找到解决方案，输出失败信息
    }
    return 0;
}
