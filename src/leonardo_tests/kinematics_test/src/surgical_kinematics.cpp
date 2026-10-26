#include "psm_controllers/common/surgical_kinematics.hpp"

#include <algorithm>

// ======================== 静态常量定义 ========================
constexpr float32 SurgicalKinematics::LA_PSM_[12];
constexpr float32 SurgicalKinematics::LA_ECM_[9];
constexpr float32 SurgicalKinematics::M_TO_MM;
constexpr float32 SurgicalKinematics::MM_TO_M;

// ======================== 私有工具函数 ========================

void SurgicalKinematics::normalizeRot(float32 &nx, float32 &ny, float32 &nz) {
    float32 d = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (std::fabs(d - 1.0F) > 0.01F) {
        nx /= d;
        ny /= d;
        nz /= d;
    }
}

float32 SurgicalKinematics::psmTargetFunc(float32 x, const Post &p) {
    const float32 y_sin = std::cos(x);
    const float32 y_cos = std::sin(x);
    return p.ny * (y_sin + 1.0F) * (p.ny - 2.0F * y_cos * y_sin) - ((p.nx - 1.0F) * y_sin + p.ny * y_cos) * (y_cos * y_cos);
}

float32 SurgicalKinematics::psmTargetFuncDeriv(float32 x, const Post &p) {
    const float32 y_sin = std::cos(x);
    const float32 y_cos = std::sin(x);
    return y_cos * y_cos * (y_cos * (p.nx - 1.0F) - p.ny * y_sin) - 2.0F * y_cos * y_sin * (y_sin * (p.nx - 1.0F) + p.ny * y_cos) -
           p.ny * (y_sin + 1.0F) * 2.0F * (y_sin * y_sin - y_cos * y_cos) - p.ny * y_cos * (p.ny - 2.0F * y_cos * y_sin);
}

bool SurgicalKinematics::resolvePsmQ4(const Post &p, float32 &q4_out) {
    float32 wucha = 2.0F;
    float32 x1 = 0.0F;

    if (std::fabs(p.ny) < 1e-6F) {
        q4_out = 0.0F;
        return true;
    }

    while (wucha > 0.0001F) {
        const float32 fx = psmTargetFunc(x1, p);
        const float32 df = psmTargetFuncDeriv(x1, p);

        if (std::fabs(df) < 0.001F) {
            q4_out = 0.0F;
            return false;
        }
        x1 = x1 - fx / df;
        wucha = std::fabs(fx);
    }
    q4_out = x1;
    return true;
}

// ======================== PSM 求解 ========================

bool SurgicalKinematics::solvePsm(const Post &post, Dof &out_dof) {
    // ✅ 【入口包装】外部输入 m -> 内部工作副本 mm
    Post work = post;
    work.iPosX *= M_TO_MM;
    work.iPosY *= M_TO_MM;
    work.iPosZ *= M_TO_MM;
    normalizeRot(work.nx, work.ny, work.nz);

    // --- 以下为核心算法，全程使用 mm 量纲，逻辑零修改 ---
    bool ik_ok = resolvePsmQ4(work, out_dof.q[4]);

    const float32 q4_sin = std::sin(out_dof.q[4]);
    const float32 q4_cos = std::cos(out_dof.q[4]);

    float32 temp_q3 = -work.nz / ((q4_cos + 1.0F) * std::sqrt((work.nx + 1.0F) / 2.0F));
    out_dof.q[3] = std::asin(temp_q3);

    const float32 q3_sin = std::sin(out_dof.q[3]);
    const float32 q3_cos = std::sqrt(1.0F - q3_sin * q3_sin);

    const float32 px = work.iPosX - (LA_PSM_[5] - LA_PSM_[8] * q3_sin * q3_sin + LA_PSM_[6] * q3_cos -
                                     (LA_PSM_[9] + LA_PSM_[11]) *
                                         (q3_cos * q4_sin * q4_sin + q4_cos * q3_sin * q3_sin - q3_cos * q3_cos * q4_cos * q4_cos) +
                                     q3_cos * q4_cos * (LA_PSM_[7] + LA_PSM_[8] * q3_cos));

    const float32 py =
        work.iPosY - (q4_sin * (LA_PSM_[7] + LA_PSM_[8] * q3_cos) + q4_sin * q4_cos * (q3_cos + 1.0F) * (LA_PSM_[9] + LA_PSM_[11]));

    const float32 pz =
        work.iPosZ - (-LA_PSM_[6] * q3_sin - q4_cos * q3_sin * (LA_PSM_[7] + LA_PSM_[8] * q3_cos) - LA_PSM_[8] * q3_cos * q3_sin -
                      q3_sin * (LA_PSM_[9] + LA_PSM_[11]) * (q4_cos + 1.0F) * (q4_cos + q3_cos * q4_cos - 1.0F));

    out_dof.q[2] = std::asin(py / LA_PSM_[3]);
    const float32 p_temp = (LA_PSM_[2] + LA_PSM_[4] + LA_PSM_[3] * std::sqrt(1.0F - std::sin(out_dof.q[2]) * std::sin(out_dof.q[2])));
    out_dof.q[1] = -std::asin(pz / p_temp);
    out_dof.q[0] = px - LA_PSM_[4] - p_temp * std::sqrt(1.0F - std::sin(out_dof.q[1]) * std::sin(out_dof.q[1]));

    out_dof.q[5] = work.DofFive;
    out_dof.q[6] = work.iEndAngle;
    // --- 核心算法结束 ---

    // ✅ 【出口包装】内部输出 mm -> 外部结果 m (仅转换位置关节)
    out_dof.q[0] *= MM_TO_M;
    out_dof.q[1] *= MM_TO_M;
    out_dof.q[2] *= MM_TO_M;

    return ik_ok;
}

// ======================== ECM 求解 ========================

bool SurgicalKinematics::solveEcm(const Post &post, Dof &out_dof) {
    // ✅ 【入口包装】外部输入 m -> 内部工作副本 mm
    Post work = post;
    work.iPosX *= M_TO_MM;
    work.iPosY *= M_TO_MM;
    work.iPosZ *= M_TO_MM;
    normalizeRot(work.nx, work.ny, work.nz);

    // --- 以下为核心算法，全程使用 mm 量纲，逻辑零修改 ---
    out_dof.q[4] = -1.0F * std::asin(work.nz);
    const float32 q4_cos = std::cos(out_dof.q[4]);

    float32 t3 = work.ny / q4_cos;
    out_dof.q[3] = std::asin(t3);

    float32 p_temp = (work.iPosY - LA_ECM_[7] * std::sin(out_dof.q[3]) - LA_ECM_[8] * work.ny) / LA_ECM_[4];
    out_dof.q[2] = std::asin(p_temp);

    p_temp = -1.0F * (work.iPosZ + LA_ECM_[8] * std::sin(out_dof.q[4])) / (LA_ECM_[3] + LA_ECM_[5] + LA_ECM_[4] * std::cos(out_dof.q[2]));
    out_dof.q[1] = std::asin(p_temp);

    out_dof.q[0] = work.iPosX - LA_ECM_[6] - LA_ECM_[7] * std::cos(out_dof.q[3]) -
                   LA_ECM_[8] * std::cos(out_dof.q[3]) * std::cos(out_dof.q[4]) - LA_ECM_[5] -
                   (LA_ECM_[3] + LA_ECM_[5] + LA_ECM_[4] * std::cos(out_dof.q[2])) * std::cos(out_dof.q[1]);

    out_dof.q[5] = work.DofFive;
    out_dof.q[6] = work.iEndAngle;
    // --- 核心算法结束 ---

    // ✅ 【出口包装】内部输出 mm -> 外部结果 m (仅转换位置关节)
    out_dof.q[0] *= MM_TO_M;
    out_dof.q[1] *= MM_TO_M;
    out_dof.q[2] *= MM_TO_M;

    return true;
}