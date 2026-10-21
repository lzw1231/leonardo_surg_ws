#include "psm_controllers/common/psm_kinematics.hpp"
#include <cmath>

namespace {
// PSM连杆参数 LA[12] 短版本
constexpr float32 LA[12] = {0.0F, 0.0F, 3.3F, 23.8F, 3.3F, 7.9F, 3.7F, 3.7F, 3.7F, 17.0F, 0.0F, 23.0F};
// ECM内窥镜连杆参数
constexpr float32 LA_ecm[9] = {0.0F, 0.0F, 0.0F, 4.6F, 26.9F, 4.6F, 4.6F, 4.6F, 51.0F};

// PSM目标函数 fff
float32 fff(float32 x, const Post &Post) {
    const float32 y_sin = std::cos(x);
    const float32 y_cos = std::sin(x);
    return Post.ny * (y_sin + 1.0F) * (Post.ny - 2.0F * y_cos * y_sin) -
           ((Post.nx - 1.0F) * y_sin + Post.ny * y_cos) * (y_cos * y_cos);
}

// PSM目标函数导数 dfff
float32 dfff(float32 x, const Post &Post) {
    const float32 y_sin = std::cos(x);
    const float32 y_cos = std::sin(x);
    return y_cos * y_cos * (y_cos * (Post.nx - 1.0F) - Post.ny * y_sin) -
           2.0F * y_cos * y_sin * (y_sin * (Post.nx - 1.0F) + Post.ny * y_cos) -
           Post.ny * (y_sin + 1.0F) * 2.0F * (y_sin * y_sin - y_cos * y_cos) - Post.ny * y_cos * (Post.ny - 2.0F * y_cos * y_sin);
}

// 牛顿迭代求解 q4
bool resolve_q4(const Post &Post, float32 &x1_out) {
    float32 wucha = 2.0F;
    float32 x1 = 0.0F;
    if (std::fabs(Post.ny) < 1e-6F) {
        x1_out = 0.0F;
        return true;
    }
    while (wucha > 0.0001F) {
        const float32 fx = fff(x1, Post);
        const float32 df = dfff(x1, Post);
        if (std::fabs(df) < 0.001F) {
            x1 = 0.0F;
            wucha = 0.0F;
            x1_out = x1;
            return false; // 奇异，导数接近0
        }
        x1 = x1 - fx / df;
        wucha = std::fabs(fx);
    }
    x1_out = x1;
    return true;
}

// 工具：限制输入在 [-1,1]，防止asin输入越界NaN
float32 clamp_asin_input(float32 v) {
    if (v > 1.0F)
        return 1.0F;
    if (v < -1.0F)
        return -1.0F;
    return v;
}
} // namespace

void normalize_rot(float32 &nx, float32 &ny, float32 &nz) {
    float32 d = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (std::fabs(d) < 1e-6F)
        return;
    nx /= d;
    ny /= d;
    nz /= d;
}

bool compute_psm(const Post &post, Dof &out_dof) {
    // 先归一姿态向量，避免输入非单位向量带来错误
    Post work_post = post;
    normalize_rot(work_post.nx, work_post.ny, work_post.nz);

    bool ret_ok = true;
    float32 q4{0.F};
    ret_ok = resolve_q4(work_post, q4);
    out_dof.q[4] = q4;

    const float32 q4_sin = std::sin(out_dof.q[4]);
    const float32 q4_cos = std::cos(out_dof.q[4]);

    // q3
    float32 temp_q3 = -work_post.nz / ((q4_cos + 1.0F) * std::sqrt((work_post.nx + 1.0F) / 2.0F));
    temp_q3 = clamp_asin_input(temp_q3);
    out_dof.q[3] = std::asin(temp_q3);

    const float32 q3_sin = std::sin(out_dof.q[3]);
    const float32 q3_cos = std::sqrt(1.0F - q3_sin * q3_sin);

    const float32 px =
        work_post.iPosX -
        (LA[5] - LA[8] * q3_sin * q3_sin + LA[6] * q3_cos -
         (LA[9] + LA[11]) * (q3_cos * q4_sin * q4_sin + q4_cos * q3_sin * q3_sin - q3_cos * q3_cos * q4_cos * q4_cos) +
         q3_cos * q4_cos * (LA[7] + LA[8] * q3_cos));

    const float32 py =
        work_post.iPosY - (q4_sin * (LA[7] + LA[8] * q3_cos) + q4_sin * q4_cos * (q3_cos + 1.0F) * (LA[9] + LA[11]));

    const float32 pz = work_post.iPosZ - (-LA[6] * q3_sin - q4_cos * q3_sin * (LA[7] + LA[8] * q3_cos) - LA[8] * q3_cos * q3_sin -
                                          q3_sin * (LA[9] + LA[11]) * (q4_cos + 1.0F) * (q4_cos + q3_cos * q4_cos - 1.0F));

    // q2
    float32 t2 = py / LA[3];
    t2 = clamp_asin_input(t2);
    out_dof.q[2] = std::asin(t2);

    const float32 p_temp = (LA[2] + LA[4] + LA[3] * std::sqrt(1.0F - std::sin(out_dof.q[2]) * std::sin(out_dof.q[2])));

    float32 t1 = pz / p_temp;
    t1 = clamp_asin_input(t1);
    out_dof.q[1] = -std::asin(t1);

    // q0
    out_dof.q[0] = px - LA[4] - p_temp * std::sqrt(1.0F - std::sin(out_dof.q[1]) * std::sin(out_dof.q[1]));

    // q5 q6 外部输入
    out_dof.q[5] = work_post.DofFive;
    out_dof.q[6] = work_post.iEndAngle;

    return ret_ok;
}

void compute_ecm(const Post &post, Dof &out_dof) {
    Post work_post = post;
    normalize_rot(work_post.nx, work_post.ny, work_post.nz);

    out_dof.q[4] = -1.0F * std::asin(work_post.nz);
    const float32 q4_cos = std::cos(out_dof.q[4]);

    float32 t3 = work_post.ny / q4_cos;
    t3 = clamp_asin_input(t3);
    out_dof.q[3] = std::asin(t3);

    float32 p_temp = (work_post.iPosY - LA_ecm[7] * std::sin(out_dof.q[3]) - LA_ecm[8] * work_post.ny) / LA_ecm[4];
    p_temp = clamp_asin_input(p_temp);
    out_dof.q[2] = std::asin(p_temp);

    p_temp = -1.0F * (work_post.iPosZ + LA_ecm[8] * std::sin(out_dof.q[4])) /
             (LA_ecm[3] + LA_ecm[5] + LA_ecm[4] * std::cos(out_dof.q[2]));
    p_temp = clamp_asin_input(p_temp);
    out_dof.q[1] = std::asin(p_temp);

    out_dof.q[0] = work_post.iPosX - LA_ecm[6] - LA_ecm[7] * std::cos(out_dof.q[3]) -
                   LA_ecm[8] * std::cos(out_dof.q[3]) * std::cos(out_dof.q[4]) - LA_ecm[5] -
                   (LA_ecm[3] + LA_ecm[5] + LA_ecm[4] * std::cos(out_dof.q[2])) * std::cos(out_dof.q[1]);

    out_dof.q[5] = work_post.DofFive;
    out_dof.q[6] = work_post.iEndAngle;
}
