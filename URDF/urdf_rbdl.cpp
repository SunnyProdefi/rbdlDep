#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Addons;

int main() {
    Model model;

    // 假设 URDF 文件位于当前目录下的 "robot.urdf"
    bool load_success = URDFReadFromFile("../../Robot/robot.urdf", &model, false);

    if (load_success) {
        std::cout << "Robot model loaded successfully!" << std::endl;
    } else {
        std::cout << "Failed to load the robot model." << std::endl;
    }

    return 0;
}
