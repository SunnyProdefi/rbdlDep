#include <iostream>
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) { 
  rbdl_check_api_version (RBDL_API_VERSION);

  Model* model = NULL; // 声明一个指向模型的指针并初始化为空

  unsigned int body_a_id, body_b_id, body_c_id; // 声明三个无符号整数变量，用于存储三个刚体的ID
  Body body_a, body_b, body_c; // 声明三个刚体
  Joint joint_a, joint_b, joint_c; // 声明三个关节

  model = new Model(); // 实例化一个新的模型对象

  model->gravity = Vector3d (0., -9.81, 0.); // 设置模型的重力向量，即地球重力

  // 初始化body_a，并创建一个绕z轴旋转的关节
  body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  joint_a = Joint(
    JointTypeRevolute,
    Vector3d (0., 0., 1.)
  );
  
  // 将body_a添加到模型中，没有父体，位于世界坐标原点
  body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);
  
  // 初始化body_b，并创建一个绕z轴旋转的关节
  body_b = Body (1., Vector3d (0., 0.5, 0.), Vector3d (1., 1., 1.));
  joint_b = Joint (
    JointTypeRevolute,
    Vector3d (0., 0., 1.)
  );
  
  // 将body_b添加到body_a，位移向量为x轴方向1单位长度
  body_b_id = model->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);
  
  // 初始化body_c，并创建一个绕z轴旋转的关节
  body_c = Body (0., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
  joint_c = Joint (
    JointTypeRevolute,
    Vector3d (0., 0., 1.)
  );
  
  // 将body_c添加到body_b，位移向量为y轴方向1单位长度
  body_c_id = model->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);
 
  VectorNd Q = VectorNd::Zero (model->q_size); // 初始化关节位置向量
  VectorNd QDot = VectorNd::Zero (model->qdot_size); // 初始化关节速度向量
  VectorNd Tau = VectorNd::Zero (model->qdot_size); // 初始化关节力矩向量
  VectorNd QDDot = VectorNd::Zero (model->qdot_size); // 初始化关节加速度向量

  ForwardDynamics (*model, Q, QDot, Tau, QDDot); // 计算前向动力学

  std::cout << QDDot.transpose() << std::endl; // 输出关节加速度向量

  delete model; // 删除模型对象，释放内存

  return 0; // 程序结束
}