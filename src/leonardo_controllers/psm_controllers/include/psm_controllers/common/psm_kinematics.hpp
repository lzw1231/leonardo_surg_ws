#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace psm_kinematics {

// ========================================================================
// PSM (psm_1 / psm_2 / psm_3) 的蛇骨连杆参数 (12 个)
inline constexpr std::array<double, 12> DEFAULT_LA_PSM = {0, 0, 3.3, 23.8, 3.3, 7.9, 3.7, 3.7, 3.7, 17, 0, 23};

// ECM (ecm_1) 的内窥镜连杆参数
inline constexpr std::array<double, 9> DEFAULT_LA_ECM = {0, 0, 0, 4.6, 26.9, 4.6, 4.6, 4.6, 51};

// ========================================================================
// 输入位姿（单位：米、弧度）
// ========================================================================
struct PoseInput {
    double x, y, z;        // 位置 (米)
    double qw, qx, qy, qz; // 四元数姿态
    double yaw;            // 对应 Post.DofFive (弧度)
    double gripper;        // 对应 Post.iEndAngle (弧度)
};

// ========================================================================
// 输出结果
// ========================================================================
struct PsmOutput {
    std::array<double, 7> q;      // 7 个关节解算角度 (弧度)
    std::array<double, 6> LA_len; // A 组拉线长度 (米)
    std::array<double, 6> LB_len; // B 组拉线长度 (米)
};

struct EcmOutput {
    std::array<double, 7> q;  // 7 个关节解算角度 (弧度)
    std::array<double, 4> dL; // 4 根内窥镜拉线的长度变化 (米)
};

// ========================================================================
// 内部纯数学辅助工具
// ========================================================================
namespace detail {
inline double sinj(double x) { return std::sin(x); }
inline double cosj(double x) { return std::cos(x); }
inline double asinj(double x) { return std::asin(std::clamp(x, -1.0, 1.0)); }
constexpr double PI = 3.141592653589793;
} // namespace detail

// ========================================================================
// 方法 1：标准 PSM 运动学
//   LA 默认为 DEFAULT_LA_PSM，可传入自定义数组覆盖
// ========================================================================
inline PsmOutput compute_psm(const PoseInput &input, const std::array<double, 12> &LA = DEFAULT_LA_PSM) {
    using namespace detail;
    PsmOutput out;

    double nx = 2.0 * (input.qx * input.qz + input.qw * input.qy);
    double ny = 2.0 * (input.qy * input.qz - input.qw * input.qx);
    double nz = 1.0 - 2.0 * (input.qx * input.qx + input.qy * input.qy);

    // 1. 牛顿迭代求 q4
    auto fff = [&](double x) {
        double y_sin = cosj(x), y_cos = sinj(x);
        return ny * (y_sin + 1.0) * (ny - 2.0 * y_cos * y_sin) - ((nx - 1.0) * y_sin + ny * y_cos) * (y_cos * y_cos);
    };
    auto dfff = [&](double x) {
        double y_sin = cosj(x), y_cos = sinj(x);
        return y_cos * y_cos * (y_cos * (nx - 1.0) - ny * y_sin) -
               2.0 * y_cos * y_sin * (y_sin * (nx - 1.0) + ny * y_cos) -
               ny * (y_sin + 1) * 2 * (y_sin * y_sin - y_cos * y_cos) - ny * y_cos * (ny - 2.0 * y_cos * y_sin);
    };
    double wucha = 2.0, x1 = 0;
    if (ny != 0) {
        while (wucha > 0.0001) {
            double fx = fff(x1), df = dfff(x1);
            if (std::abs(df) < 0.001) {
                x1 = 0;
                break;
            } else {
                x1 = x1 - fx / df;
                wucha = std::abs(fx);
            }
        }
    }
    out.q[4] = x1;

    // 2. 解析 q3
    double q4_sin = sinj(out.q[4]), q4_cos = cosj(out.q[4]);
    out.q[3] = asinj(-nz / ((q4_cos + 1) * std::sqrt((nx + 1) / 2)));

    // 3. 解析 q0, q1, q2（用传入的 LA）
    double q3_sin = sinj(out.q[3]), q3_cos = std::sqrt(1 - q3_sin * q3_sin);
    double px =
        input.x -
        (LA[5] - LA[8] * q3_sin * q3_sin + LA[6] * q3_cos -
         (LA[9] + LA[11]) * (q3_cos * q4_sin * q4_sin + q4_cos * q3_sin * q3_sin - q3_cos * q3_cos * q4_cos * q4_cos) +
         q3_cos * q4_cos * (LA[7] + LA[8] * q3_cos));
    double py = input.y - (q4_sin * (LA[7] + LA[8] * q3_cos) + q4_sin * q4_cos * (q3_cos + 1) * (LA[9] + LA[11]));
    double pz = input.z - (LA[6] * q3_sin - q4_cos * q3_sin * (LA[7] + LA[8] * q3_cos) - LA[8] * q3_cos * q3_sin -
                           q3_sin * (LA[9] + LA[11]) * (q4_cos + 1) * (q4_cos + q3_cos * q4_cos - 1));

    out.q[2] = asinj(py / LA[3]);
    double p_temp = LA[2] + LA[4] + LA[3] * std::sqrt(1 - sinj(out.q[2]) * sinj(out.q[2]));
    out.q[1] = -asinj(pz / p_temp);
    out.q[0] = px - LA[4] - p_temp * std::sqrt(1 - sinj(out.q[1]) * sinj(out.q[1]));
    out.q[5] = input.yaw;
    out.q[6] = input.gripper;

    // 4. 计算拉线长度 (纯几何长度，单位：米)
    double LA_sum[6] = {0}, LB_sum[6] = {0};
    double r = 3.1;

    auto jointj = [&](int num, double *l_a, double *l_b) {
        static const double dv[6] = {1.63, 1.48, 1.23, 1.23, 1.48, 1.63};
        static const double dv1[6] = {0.41, 0.82, 1.09, 1.09, 0.82, 0.41};
        static const double dv2[6] = {1.42, 1.12, 0.74, 0.74, 1.12, 1.42};
        static const double dv3[6] = {1.18, 1.4, 1.56, 1.56, 1.4, 1.18};
        static const double dv4[6] = {0.4363, 0.7854, 1.1345, 2.0071, 2.3562, 2.7053};
        static const double dv5[6] = {3.5779, 0.7854, 1.1345, 5.1487, 2.3562, 2.7053};
        static const double dv6[6] = {0.0, 1.5, 1.5, 0.0, 1.5, 1.5};
        static const double dv7[6] = {1.5708, 0.7854, 1.1345, 1.5708, 2.3562, 2.7053};
        static const double dv8[6] = {0.0, 0.7854, 1.1345, 0.0, 2.3562, 2.7053};
        static const signed char iv1[6] = {0, 1, 1, 0, 1, 1};

        double T[16], ang[6] = {0}, dpi[6] = {0}, dqi[6] = {0};
        int Fuy = 1, numq = 1;
        int FuHao = ((num == 3) || (num == 4)) ? -1 : 1;

        if (num == 1) {
            for (int i = 0; i < 6; i++) {
                dpi[i] = dv[i];
                dqi[i] = dv2[i];
                ang[i] = dv4[i];
            }
            Fuy = 1;
            numq = 1;
        } else if (num == 2) {
            for (int i = 0; i < 6; i++) {
                dpi[i] = dv1[i];
                dqi[i] = dv3[i];
                ang[i] = dv4[i];
            }
            Fuy = 2;
            numq = 2;
        } else if (num == 3) {
            for (int i = 0; i < 6; i++) {
                dpi[i] = dv1[i];
                dqi[i] = dv3[i];
                ang[i] = dv5[i];
            }
            Fuy = 2;
            numq = 2;
        } else if (num == 4) {
            for (int i = 0; i < 6; i++) {
                dpi[i] = dv[i];
                dqi[i] = dv2[i];
                ang[i] = dv5[i];
            }
            Fuy = 1;
            numq = 1;
        } else if (num == 5 || num == 7) {
            for (int i = 0; i < 6; i++) {
                dpi[i] = iv1[i];
                dqi[i] = dv6[i];
                ang[i] = dv7[i];
            }
            Fuy = 1;
            numq = 3;
        } else if (num == 6 || num == 8) {
            for (int i = 0; i < 6; i++) {
                dpi[i] = iv1[i];
                dqi[i] = dv6[i];
                ang[i] = dv8[i];
            }
            Fuy = 2;
            numq = 4;
        }

        if (Fuy == 1) {
            T[0] = cosj(out.q[numq]);
            T[4] = 0.0;
            T[8] = FuHao * sinj(out.q[numq]);
            T[12] = 0.0;
            T[2] = -FuHao * sinj(out.q[numq]);
            T[6] = 0.0;
            T[10] = cosj(out.q[numq]);
            T[14] = 0.0;
            T[1] = 0;
            T[3] = 0;
            T[5] = 1;
            T[7] = 0;
            T[9] = 0;
            T[11] = 0;
            T[13] = 0;
            T[15] = 1;
        } else {
            T[0] = cosj(out.q[numq]);
            T[4] = -FuHao * sinj(out.q[numq]);
            T[8] = 0.0;
            T[12] = 0.0;
            T[1] = FuHao * sinj(out.q[numq]);
            T[5] = cosj(out.q[numq]);
            T[9] = 0.0;
            T[13] = 0.0;
            T[2] = 0;
            T[3] = 0;
            T[6] = 0;
            T[7] = 0;
            T[10] = 1;
            T[11] = 0;
            T[14] = 0;
            T[15] = 1;
        }

        for (int i = 0; i < 6; i++) {
            double c_tmp = -r * sinj(ang[i]), b_tmp = r * cosj(ang[i]);
            double y[4];
            for (int k = 0; k < 4; k++)
                y[k] = ((T[k] * dqi[i] + T[k + 4] * c_tmp) + T[k + 8] * b_tmp) + T[k + 12];
            double a = -dpi[i] - y[0], b = c_tmp - y[1], cc = b_tmp - y[2];
            l_a[i] = std::sqrt(a * a + b * b + cc * cc) - dpi[i] - dqi[i];

            c_tmp = -r * sinj(ang[i] + PI);
            b_tmp = r * cosj(ang[i] + PI);
            for (int k = 0; k < 4; k++)
                y[k] = ((T[k] * dqi[i] + T[k + 4] * c_tmp) + T[k + 8] * b_tmp) + T[k + 12];
            a = -dpi[i] - y[0];
            b = c_tmp - y[1];
            cc = b_tmp - y[2];
            l_b[i] = std::sqrt(a * a + b * b + cc * cc) - dpi[i] - dqi[i];
        }
    };

    for (int Nj = 1; Nj < 9; Nj++) {
        double l_a[6], l_b[6];
        jointj(Nj, l_a, l_b);
        for (int i = 0; i < 6; i++) {
            LA_sum[i] += l_a[i];
            LB_sum[i] += l_b[i];
        }
    }

    for (int i = 0; i < 6; i++) {
        out.LA_len[i] = LA_sum[i];
        out.LB_len[i] = LB_sum[i];
    }

    return out;
}

// ========================================================================
// 方法 2：内窥镜 ECM 运动学
//   LA 默认为 DEFAULT_LA_ECM (9 个)，可传入自定义数组覆盖
// ========================================================================
inline EcmOutput compute_ecm(const PoseInput &input, const std::array<double, 9> &LA = DEFAULT_LA_ECM) {
    using namespace detail;
    EcmOutput out;

    double nx = 2.0 * (input.qx * input.qz + input.qw * input.qy);
    double ny = 2.0 * (input.qy * input.qz - input.qw * input.qx);
    double nz = 1.0 - 2.0 * (input.qx * input.qx + input.qy * input.qy);

    // 1. 求解关节角（用传入的 LA）
    out.q[4] = -1.0 * asinj(nz);
    double q4_cos = cosj(out.q[4]);
    out.q[3] = asinj(ny / q4_cos);

    double p_temp = (input.y - LA[7] * sinj(out.q[3]) - LA[8] * ny) / LA[4];
    out.q[2] = asinj(p_temp);

    p_temp = -1.0 * (input.z + LA[8] * sinj(out.q[4])) / (LA[3] + LA[5] + LA[4] * cosj(out.q[2]));
    out.q[1] = asinj(p_temp);

    out.q[0] = input.x - LA[6] - LA[7] * cosj(out.q[3]) - LA[8] * cosj(out.q[3]) * cosj(out.q[4]) - LA[5] -
               (LA[3] + LA[5] + LA[4] * cosj(out.q[2])) * cosj(out.q[1]);
    out.q[5] = input.yaw;
    out.q[6] = input.gripper;

    // 2. 计算 4 根内窥镜拉线变化 (纯几何长度变化量)
    static const double XWJ[4] = {0.4189, 1.1519, 0.7854, 0.7854};
    double r_n_kui_jing = 4;

    out.dL[0] = 2.0 * r_n_kui_jing * (cosj(XWJ[0]) * sinj(out.q[1]) + sinj(XWJ[0]) * sinj(out.q[2]));
    out.dL[1] = 2.0 * r_n_kui_jing * (-1.0 * cosj(XWJ[1]) * sinj(out.q[1]) + sinj(XWJ[1]) * sinj(out.q[2]));
    out.dL[2] = 2.0 * r_n_kui_jing * (sinj(XWJ[2]) * sinj(out.q[3]) + cosj(XWJ[2]) * sinj(out.q[4]));
    out.dL[3] = 2.0 * r_n_kui_jing * (sinj(XWJ[3]) * sinj(out.q[3]) - cosj(XWJ[3]) * sinj(out.q[4]));

    return out;
}

} // namespace psm_kinematics