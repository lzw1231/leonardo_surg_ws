#pragma once

#include <cmath>
#include <cstdint>

namespace kinematics_test {

using float32 = float;

/**
 * @brief 末端期望位姿结构体 (对应文档 Post)
 * @note 对外接口单位: 位置(m), 姿态(无量纲), 关节角(rad)
 */
struct Post {
    float32 iPosX{0.F}; // m
    float32 iPosY{0.F}; // m
    float32 iPosZ{0.F}; // m
    float32 nx{0.F};    // 旋转矩阵列向量
    float32 ny{0.F};
    float32 nz{0.F};
    float32 DofFive{0.F};   // q5 (rad)
    float32 iEndAngle{0.F}; // q6 (rad)
};

/**
 * @brief 输出关节角度 q[0] ~ q[6] (对应文档 Dof)
 * @note 对外接口单位: 位置(m), 角度(rad)
 */
struct Dof {
    float32 q[7]{0.F};
};

/**
 * @brief 手术机器人逆运动学求解器
 * @note 包含 PSM(器械) 与 ECM(内窥镜) 两套算法
 * @note 对外 I/O 单位已统一封装为: 位置(m), 角度(rad)
 * @note 内部核心算法仍基于原始文档参数(mm)运行
 */
class SurgicalKinematics {
  public:
    SurgicalKinematics() = default;
    ~SurgicalKinematics() = default;

    SurgicalKinematics(const SurgicalKinematics &) = delete;
    SurgicalKinematics &operator=(const SurgicalKinematics &) = delete;

    /**
     * @brief PSM主机构(带夹爪/器械)逆运动学
     * @param[in] post 输入期望末端位姿 (m, rad)
     * @param[out] out_dof 输出关节变量 q0~q6 (m, rad)
     * @return true=求解正常; false=牛顿迭代奇异/导数接近0
     */
    bool solvePsm(const Post &post, Dof &out_dof);

    /**
     * @brief ECM内窥镜机构逆运动学
     * @param[in] post 输入期望末端位姿 (m, rad)
     * @param[out] out_dof 输出关节变量 q0~q6 (m, rad)
     */
    bool solveEcm(const Post &post, Dof &out_dof);

  private:
    // ==================== 内部连杆长度参数 (mm) ====================
    static constexpr float32 LA_PSM_[12] = {0.0F, 0.0F, 4.0F, 22.3F, 4.0F, 8.0F, 4.0F, 4.0F, 4.0F, 20.0F, 0.0F, 23.0F};
    static constexpr float32 LA_ECM_[9] = {0.0F, 0.0F, 0.0F, 4.6F, 27.7F, 4.6F, 4.6F, 4.6F, 55.3F};

    // 单位转换常量
    static constexpr float32 M_TO_MM = 1000.0F;
    static constexpr float32 MM_TO_M = 0.001F;

    static void normalizeRot(float32 &nx, float32 &ny, float32 &nz);
    static float32 psmTargetFunc(float32 x, const Post &p);
    static float32 psmTargetFuncDeriv(float32 x, const Post &p);
    static bool resolvePsmQ4(const Post &p, float32 &q4_out);
};

} // namespace kinematics_test