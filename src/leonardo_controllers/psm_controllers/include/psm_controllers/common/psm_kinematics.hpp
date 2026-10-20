#pragma once

#include <cmath>
#include <cstdint>

using float32 = float;

/**
 * @brief 末端期望位姿结构体，对应原代码Post
 */
struct RobotPost {
    float32 iPosX{0.F};
    float32 iPosY{0.F};
    float32 iPosZ{0.F};

    float32 nx{0.F};
    float32 ny{0.F};
    float32 nz{0.F};

    float32 DofFive{0.F};   // q5外部输入
    float32 iEndAngle{0.F}; // q6外部输入
};

/**
 * @brief 输出关节角度 q[0] ~ q[6]
 */
struct RobotDof {
    float32 q[7]{0.F};
};

/**
 * @brief PSM主机构逆运动学：末端位姿求解关节角
 * @param[in] post 输入期望末端位姿
 * @param[out] out_dof 输出关节角度 q0~q6
 * @return true求解正常; false牛顿迭代奇异/导数接近0
 */
bool compute_psm(const RobotPost &post, RobotDof &out_dof);

/**
 * @brief ECM内窥镜机构逆运动学：末端位姿求解关节角
 * @param[in] post 输入期望末端位姿
 * @param[out] out_dof 输出关节角度 q0~q6
 */
void compute_ecm(const RobotPost &post, RobotDof &out_dof);

/**
 * @brief 姿态nx/ny/nz向量归一化，防止输入模长不等于1
 */
void normalize_rot(float32 &nx, float32 &ny, float32 &nz);
