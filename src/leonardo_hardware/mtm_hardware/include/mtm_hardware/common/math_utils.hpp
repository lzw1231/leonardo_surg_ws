#ifndef RAPHAEL_HARDWARE_COMMON_MATH_UTILS_HPP_
#define RAPHAEL_HARDWARE_COMMON_MATH_UTILS_HPP_

#include <vector>
#include <algorithm>
#include <cstddef>

namespace hw_math{
    /**
     * @brief 计算二维上三角矩阵元素在一维扁平化数组中的索引
     *
     * @param idx_row 行索引
     * @param idx_col 列索引
     * @param dim 矩阵维度 (默认为 6)
     * @return unsigned int 一维数组中的索引位置
     * @note 自动处理对称性，保证 i <= j。
     *       内存存储顺序为：(0,0), (0,1)...(0,n-1), (1,1)...(1,n-1), ...
     */
    inline unsigned int flattened_index_from_triangular_index(unsigned int idx_row, unsigned int idx_col, unsigned int dim = 6) {
        // 确保 i <= j，利用标准库函数替代手写的 if 交换逻辑
        unsigned int i = std::min(idx_row, idx_col);
        unsigned int j = std::max(idx_row, idx_col);

        // 上三角矩阵扁平化数学公式
        return i * (2 * dim - i - 1) / 2 + j;
    }

    /**
     * @brief 调整向量大小并使用指定值填充所有元素
     *
     * @tparam T 向量元素类型 (如 double, float, int 等)
     * @param vec 目标向量引用
     * @param new_size 调整后的新大小
     * @param init_val 用于填充的初始值
     * @note 使用 std::vector::assign 替代 resize + std::fill。
     *       assign 语义更明确，且避免了 resize 带来的默认构造开销，性能更优。
     */
    template <typename T>
    inline void resize_and_fill(std::vector<T>& vec, std::size_t new_size, const T& init_val) {
        vec.assign(new_size, init_val);
    }
} // namespace hw_math


#endif // RAPHAEL_HARDWARE_COMMON_MATH_UTILS_HPP_
