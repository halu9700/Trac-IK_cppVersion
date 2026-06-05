#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <random>

#include "trac_ik.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Degree to radian conversion
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// Create IRB 2600 6-DOF robot arm using Modified DH parameters
KDL::Chain createIRB2600() {
    KDL::Chain chain;

    // 关节1
    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::None),
        KDL::Frame::DH_Craig1989(
            0,              // a0 = 0
            deg2rad(0),     // alpha0 = 0°
            445,          // d1 = 0.445 m
            0               // theta1 offset = 0
        )
    ));

    // 关节2
    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(
            150,              // a1 = 0.150 m
            deg2rad(-90),       // alpha1 = -90°
            0,                  // d2 = 0
            deg2rad(90)         // theta2 offset = 90°
        )
    ));

    // 关节3
    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(
            -700,             // a2 = -0.700 m
            deg2rad(0),         // alpha2 = 0°
            0,                  // d3 = 0
            0                   // theta3 offset = 0
        )
    ));

    // 关节4
    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(
            -115,             // a3 = -0.115 m
            deg2rad(90),        // alpha3 = 90°
            0.795,              // d4 = 0.795 m
            0                   // theta4 offset = 0
        )
    ));

    // 关节5
    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(
            0,                  // a4 = 0
            deg2rad(-90),       // alpha4 = -90°
            0,                  // d5 = 0
            0                   // theta5 offset = 0
        )
    ));

    // 关节6
    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(
            0,                  // a5 = 0
            deg2rad(90),        // alpha5 = 90°
            85,              // d6 = 0.085 m
            0                   // theta6 offset = 0
        )
    ));

    chain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(
            0,                  // a5 = 0
            deg2rad(0),        // alpha5 = 90°
            0,              // d6 = 0.085 m
            0                   // theta6 offset = 0
        )
    ));

    return chain;
}

KDL::Chain abbRobot() {
    // Craig修改DH参数表
    // | i | α_{i-1}(°) | a_{i-1}(mm) | d_i(mm) | θ_i     |
    // |---|------------|------------|---------|---------|
    // | 1 | 0          | 0          | 445     | θ₁      |
    // | 2 | -90        | 150        | 0       | θ₂+90   |
    // | 3 | 0          | -700       | 0       | θ₃      |
    // | 4 | 90         | -115       | 795     | θ₄      |
    // | 5 | -90        | 0          | 0       | θ₅      |
    // | 6 | 90         | 0          | 85      | θ₆      |
    const int JOINT_NUM = 6;
    double PI2 = KDL::PI / 2;
    double alpha_mdh[JOINT_NUM] = {   0, -PI2,     0,   PI2,  -PI2,  PI2};
    double a_mdh[JOINT_NUM]     = {   0,  150,  -700,  -115,     0,    0};
    double d_mdh[JOINT_NUM]     = { 445,    0,     0,   795,     0,   85};
    double theta_mdh[JOINT_NUM] = {   0,  PI2,     0,     0,     0,    0};

    KDL::Chain abb;
    for (int i = 0; i < JOINT_NUM; i++) {

        auto frame = KDL::Frame::DH_Craig1989(a_mdh[i], alpha_mdh[i], d_mdh[i], theta_mdh[i]);
        auto joint = KDL::Joint(KDL::Joint::RotZ);
        if (i == 0)
            joint = KDL::Joint(KDL::Joint::None);
        abb.addSegment(KDL::Segment("joint"+ std::to_string(i+1), joint, frame));
    }
    abb.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                KDL::Frame::DH_Craig1989(0, 0.0, 0.0, 0.0)));
    return abb;
}


// 验证给定关节角度的正解
KDL::Frame computeFK(const KDL::Chain& chain, const KDL::JntArray& q) {
    KDL::Frame cartpos;
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    fksolver.JntToCart(q, cartpos);
    return cartpos;
}

// Print joint angles in both radians and degrees
void printJntArray(const std::string& label, const KDL::JntArray& q) {
    std::cout << label << " (rad): [";
    for (unsigned int i = 0; i < q.rows(); i++) {
        std::cout << std::fixed << std::setprecision(6) << q(i);
        if (i < q.rows() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << label << " (deg): [";
    for (unsigned int i = 0; i < q.rows(); i++) {
        std::cout << std::fixed << std::setprecision(2) << rad2deg(q(i));
        if (i < q.rows() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

// Print pose
void printFrame(const std::string& label, const KDL::Frame& f) {
    std::cout << label << ":" << std::endl;
    std::cout << "  Position (m): ["
              << std::fixed << std::setprecision(6)
              << f.p.x() << ", "
              << f.p.y() << ", "
              << f.p.z() << "]" << std::endl;

    double roll, pitch, yaw;
    f.M.GetRPY(roll, pitch, yaw);
    std::cout << "  RPY (rad): ["
              << roll << ", "
              << pitch << ", "
              << yaw << "]" << std::endl;
    std::cout << "  RPY (deg): ["
              << rad2deg(roll) << ", "
              << rad2deg(pitch) << ", "
              << rad2deg(yaw) << "]" << std::endl;
}

// === 缺失的函数：计算位置误差 ===
double positionError(const KDL::Frame& f1, const KDL::Frame& f2) {
    KDL::Vector diff = f1.p - f2.p;
    return diff.Norm();
}

// === 缺失的函数：计算姿态误差（旋转角度） ===
double rotationError(const KDL::Frame& f1, const KDL::Frame& f2) {
    // 计算两个旋转矩阵之间的差异
    KDL::Rotation r_diff = f1.M.Inverse() * f2.M;

    // 从旋转矩阵获取旋转角度（轴角表示法）
    double angle = std::acos(std::min(1.0, std::max(-1.0,
        (r_diff.data[0] + r_diff.data[4] + r_diff.data[8] - 1.0) / 2.0)));

    return angle;
}

// 可选：更精确的旋转误差计算（使用四元数）
double rotationErrorQuaternion(const KDL::Frame& f1, const KDL::Frame& f2) {
    double q1_x, q1_y, q1_z, q1_w;
    double q2_x, q2_y, q2_z, q2_w;

    f1.M.GetQuaternion(q1_x, q1_y, q1_z, q1_w);
    f2.M.GetQuaternion(q2_x, q2_y, q2_z, q2_w);

    // 计算四元数点积
    double dot = q1_x*q2_x + q1_y*q2_y + q1_z*q2_z + q1_w*q2_w;

    // 确保点积在[-1,1]范围内
    dot = std::min(1.0, std::max(-1.0, dot));

    // 计算角度差（弧度）
    double angle = 2.0 * std::acos(std::abs(dot));

    return angle;
}

int main(int argc, char** argv) {
    std::cout << "IRB 2600 Inverse Kinematics Test (TRAC-IK)\n";
    std::cout << "============================================\n\n";

    // Build the IRB 2600 kinematic chain
    KDL::Chain chain = abbRobot();
    unsigned int nj = chain.getNrOfJoints();

    std::cout << "Robot: ABB IRB 2600" << std::endl;
    std::cout << "DOF:   " << nj << std::endl;

    // 第一步：验证给定关节角度的正解
    std::cout << "\nStep 1: Forward Kinematics Test\n";
    std::cout << "--------------------------------\n";

    KDL::JntArray q_test(nj);
    // 设置你在仿真环境中的关节角度 [30, 20, -10, 45, -30, 60] (度)
    q_test(0) = deg2rad(30);   // θ1
    q_test(1) = deg2rad(20);   // θ2
    q_test(2) = deg2rad(-10);  // θ3
    q_test(3) = deg2rad(45);   // θ4
    q_test(4) = deg2rad(-30);  // θ5
    q_test(5) = deg2rad(60);   // θ6

    q_test(0) = deg2rad(-4.95);   // θ1
    q_test(1) = deg2rad(-4.42);   // θ2
    q_test(2) = deg2rad(4.43);  // θ3
    q_test(3) = deg2rad(0.78);   // θ4
    q_test(4) = deg2rad(89.86);  // θ5
    q_test(5) = deg2rad(89.31);   // θ6

    printJntArray("Test joint angles", q_test);

    // 计算正解
    KDL::Frame fk_result = computeFK(chain, q_test);
    printFrame("FK result", fk_result);


    // 第二步：设置关节范围（注意关节2要考虑偏移量）
    std::cout << "\nStep 2: Setting joint limits\n";
    std::cout << "----------------------------\n";

    KDL::JntArray q_min(nj), q_max(nj);

    // 关节1: -180° ~ 180°
    q_min(0) = deg2rad(-180.0);  q_max(0) = deg2rad(180.0);

    // 关节2: 原始范围 -42° ~ 85°，加上偏移90°，所以DH空间范围是 [-42+90, 85+90] = [48°, 175°]
    // q_min(1) = deg2rad(48.0);    q_max(1) = deg2rad(175.0);
    q_min(1) = deg2rad(-42);    q_max(1) = deg2rad(85.0);
    // 关节3: -20° ~ 120°
    q_min(2) = deg2rad(-20.0);   q_max(2) = deg2rad(120.0);

    // 关节4: -300° ~ 300°
    q_min(3) = deg2rad(-300.0);  q_max(3) = deg2rad(300.0);

    // 关节5: -120° ~ 120°
    q_min(4) = deg2rad(-120.0);  q_max(4) = deg2rad(120.0);

    // 关节6: -360° ~ 360°
    q_min(5) = deg2rad(-360.0);  q_max(5) = deg2rad(360.0);

    // 打印关节范围（转换为用户角度显示）
    std::cout << "Joint limits (user angles in degrees):\n";
    for (unsigned int i = 0; i < nj; i++) {
        double min_user = rad2deg(q_min(i));
        double max_user = rad2deg(q_max(i));
        std::cout << "  Joint " << i+1 << ": [" << min_user << ", " << max_user << "]\n";
    }

    // 第三步：使用TRAC-IK求解逆解
    std::cout << "\nStep 3: Inverse Kinematics Test\n";
    std::cout << "-------------------------------\n";

    // 创建TRAC-IK求解器
    TRAC_IK::TRAC_IK ik_solver(chain, q_min, q_max, 1, 1e-4, TRAC_IK::TRAC_IK::Distance);

    // 使用正解得到的位姿作为目标
    KDL::Frame target_pose = fk_result;
    printFrame("Target pose", target_pose);

    // 初始猜测（使用当前角度）
    KDL::JntArray q_init(nj);
    // q_init = q_test;  // 使用测试角度作为初始猜测
    for (unsigned int i = 0; i < nj; i++) q_init(i) = 0.0;

    // 设置容差
    KDL::Twist bounds(KDL::Vector(0.0001, 0.0001, 0.0001),   // 位置容差 (1mm)
                     KDL::Vector(0.001, 0.001, 0.001));   // 姿态容差 (约0.057°)

    // 求解
    KDL::JntArray q_result(nj);
    int result = ik_solver.CartToJnt(q_init, target_pose, q_result, bounds);

    if (result >= 0) {
        std::cout << "\n✓ IK solution found!\n";
        printJntArray("Solution", q_result);

        // 验证求解结果
        KDL::Frame verify_pose = computeFK(chain, q_result);
        printFrame("Verification FK", verify_pose);

        // 计算误差
        double pos_error = positionError(target_pose, verify_pose);
        double rot_error = rotationError(target_pose, verify_pose);
        double rot_error_quat = rotationErrorQuaternion(target_pose, verify_pose);

        std::cout << "\nError analysis:\n";
        std::cout << "  Position error: " << pos_error * 1000 << " mm\n";
        std::cout << "  Rotation error (matrix): " << rad2deg(rot_error) << " deg\n";
        std::cout << "  Rotation error (quaternion): " << rad2deg(rot_error_quat) << " deg\n";

        // 比较输入和输出
        std::cout << "\nComparison with input angles:\n";
        std::cout << "Joint\tInput(deg)\tOutput(deg)\tDiff(deg)\n";
        for (unsigned int i = 0; i < nj; i++) {
            double input_deg = rad2deg(q_test(i));
            double output_deg = rad2deg(q_result(i));
            std::cout << i+1 << "\t"
                      << std::fixed << std::setprecision(2) << input_deg << "\t\t"
                      << output_deg << "\t\t"
                      << std::abs(input_deg - output_deg) << "\n";
        }
    } else {
        std::cout << "\n✗ No IK solution found.\n";

        // 如果无解，尝试不同的初始猜测
        std::cout << "\nTrying with different initial guesses...\n";

        // 尝试多个随机初始猜测
        std::random_device rd;
        std::mt19937 gen(rd());

        bool found = false;
        for (int trial = 0; trial < 20 && !found; trial++) {
            for (unsigned int i = 0; i < nj; i++) {
                // 生成随机初始猜测
                std::uniform_real_distribution<> dis(0.0, 1.0);
                double t = dis(gen);
                q_init(i) = q_min(i) + t * (q_max(i) - q_min(i));
            }

            result = ik_solver.CartToJnt(q_init, target_pose, q_result, bounds);
            if (result >= 0) {
                found = true;
                std::cout << "\n✓ Found solution with random initial guess #" << trial+1 << "\n";
                printJntArray("Solution", q_result);

                // 验证误差
                KDL::Frame verify_pose = computeFK(chain, q_result);
                double pos_error = positionError(target_pose, verify_pose);
                double rot_error = rotationError(target_pose, verify_pose);

                std::cout << "Verification error: " << pos_error * 1000 << " mm, "
                          << rad2deg(rot_error) << " deg\n";
                break;
            }
        }

        if (!found) {
            std::cout << "\n✗ Still no solution found after 20 random attempts.\n";
        }
    }

    std::cout << "\nAll tests completed.\n";
    return 0;
}