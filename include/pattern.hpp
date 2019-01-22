/**
 * @file pattern.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 产生和brief描述子计算相关的"模式""
 * @version 0.1
 * @date 2019-01-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _SSVO_PATTERN_HPP_
#define _SSVO_PATTERN_HPP_

#include <array>

namespace ssvo{

/**
 * @brief 定义"模式"结构体
 * @detials 模式定义了在计算特征点的brief描述子的时候,参考它周围的哪些图像像素点
 * @tparam T 每个元素的数据类型
 * @tparam N 参考的点的数目
 * @tparam S 模式的大小
 */
template <typename T, int N, int S>
struct Pattern
{
    ///一些参数
    enum{
        Num = N,                ///<参考的点的数目
        Size = S,               ///<模式的大小
        SizeWithBorder = S+2    ///<考虑边框的时候,模式区域的大小
    };

    /// TODO ?
    const std::array<std::array<int, 2>, N> data;
    /// TODO???  
    std::array<std::array<int, 5>, N> offset;
    /**
     * @brief 获取模式
     * 
     * @param[in] mat           TODO 
     * @param[out] pattern      得到的模式? TODO 
     * @param[out] gx           TODO 
     * @param[out] gy           TODO 
     */
    inline void getPattern(Matrix<T, SizeWithBorder, SizeWithBorder, RowMajor> &mat,
                           Matrix<T, N, 1> &pattern,
                           Matrix<T, N, 1> &gx,
                           Matrix<T, N, 1> &gy) const
    {
        const T* mat_ptr = mat.data();
        for(int i = 0; i < N; i++)
        {
            const std::array<int, 5> &ofs = offset[i];
            pattern[i] = mat_ptr[ofs[0]];
            gx[i] = (mat_ptr[ofs[1]]-mat_ptr[ofs[2]])*0.5;
            gy[i] = (mat_ptr[ofs[3]]-mat_ptr[ofs[4]])*0.5;
        }
    }

    /**
     * @brief 获取模式
     * 
     * @param[in] mat           TODO 
     * @param[out] pattern       得到的模式
     */
    inline void getPattern(Matrix<T, SizeWithBorder, SizeWithBorder, RowMajor> &mat, Matrix<T, N, 1> &pattern) const
    {
        const T* mat_ptr = mat.data();
        for(int i = 0; i < N; i++)
        {
            pattern[i] = mat_ptr[offset[i][0]];
        }
    }

    /**
     * @brief 构造函数
     * 
     * @param[in] array 原始元素
     */
    Pattern(std::array<std::array<int, 2>, N> array):
        data(array)
    {
        for(int i = 0; i < N; ++i)
        {
            assert(abs(data[i][0]) <= S && abs(data[i][1]) <= S);
            int index = (data[i][0]+SizeWithBorder/2) + (data[i][1]+SizeWithBorder/2) * SizeWithBorder;
            offset[i] = {index, index+1, index-1, index+SizeWithBorder, index-SizeWithBorder};
        }
    }
};

///pattern0
const Pattern<float, 64, 8> pattern0(
    {{
         {-4, -4}, {-3, -4}, {-2, -4}, {-1, -4}, { 0, -4}, { 1, -4}, { 2, -4}, { 3, -4},
         {-4, -3}, {-3, -3}, {-2, -3}, {-1, -3}, { 0, -3}, { 1, -3}, { 2, -3}, { 3, -3},
         {-4, -2}, {-3, -2}, {-2, -2}, {-1, -2}, { 0, -2}, { 1, -2}, { 2, -2}, { 3, -2},
         {-4, -1}, {-3, -1}, {-2, -1}, {-1, -1}, { 0, -1}, { 1, -1}, { 2, -1}, { 3, -1},
         {-4,  0}, {-3,  0}, {-2,  0}, {-1,  0}, { 0,  0}, { 1,  0}, { 2,  0}, { 3,  0},
         {-4,  1}, {-3,  1}, {-2,  1}, {-1,  1}, { 0,  1}, { 1,  1}, { 2,  1}, { 3,  1},
         {-4,  2}, {-3,  2}, {-2,  2}, {-1,  2}, { 0,  2}, { 1,  2}, { 2,  2}, { 3,  2},
         {-4,  3}, {-3,  3}, {-2,  3}, {-1,  3}, { 0,  3}, { 1,  3}, { 2,  3}, { 3,  3},
     }}
);

///pattern1
const Pattern<float, 16, 7> pattern1(
    {{
         {-1, -3}, { 0, -3}, { 1, -3},
         {-2, -2}, { 2, -2},
         {-3, -1}, { 3, -1},
         {-3,  0}, { 3,  0},
         {-3,  1}, { 3,  1},
         {-2,  2}, { 2,  2},
         {-1,  3}, { 0,  3}, { 1,  3},
     }}
);

///pattern2
const Pattern<float, 25, 7> pattern2(
    {{
         {-3, -3}, {-1, -3}, { 0, -3}, { 1, -3}, { 3, -3},
         {-2, -2}, { 2, -2},
         {-3, -1}, {-1, -1}, { 1, -1}, { 3, -1},
         {-3,  0}, { 0,  0}, { 3,  0},
         {-3,  1}, {-1,  1}, { 1,  1}, { 3,  1},
         {-2,  2}, { 2,  2},
         {-3,  3}, {-1,  3}, { 0,  3}, { 1,  3}, { 3,  3},
     }}
);

///pattern3
const Pattern<float, 31, 7> pattern3(
    {{
         {-1, -3}, { 0, -3}, { 1, -3},
         {-2, -2}, { 0, -2}, { 2, -2},
         {-3, -1}, { 0, -1}, { 3, -1},
         {-3,  0}, {-2,  0}, {-1,  0}, { 0,  0}, { 1,  0}, { 2,  0}, { 3,  0},
         {-3,  1}, { 0,  1}, { 3,  1},
         {-2,  2}, { 0,  2}, { 2,  2},
         {-1,  3}, { 0,  3}, { 1,  3},
     }}
);
///pattern4
const Pattern<float, 32, 8> pattern4(
    {{
         {-4, -4}, {-2, -4}, { 0, -4}, { 2, -4},
         {-3, -3}, {-1, -3}, { 1, -3}, { 3, -3},
         {-4, -2}, {-2, -2}, { 0, -2}, { 2, -2},
         {-3, -1}, {-1, -1}, { 1, -1}, { 3, -1},
         {-4,  0}, {-2,  0}, { 0,  0}, { 2,  0},
         {-3,  1}, {-1,  1}, { 1,  1}, { 3,  1},
         {-4,  2}, {-2,  2}, { 0,  2}, { 2,  2},
         {-3,  3}, {-1,  3}, { 1,  3}, { 3,  3},
     }}
);
///pattern5
const Pattern<float, 49, 13> pattern5(
    {{
         {-6, -6}, {-4, -6}, {-2, -6}, { 0, -6}, { 2, -6}, { 4, -6}, { 6, -6},
         {-6, -4}, {-4, -4}, {-2, -4}, { 0, -4}, { 2, -4}, { 4, -4}, { 6, -4},
         {-6, -2}, {-4, -2}, {-2, -2}, { 0, -2}, { 2, -2}, { 4, -2}, { 6, -2},
         {-6,  0}, {-4,  0}, {-2,  0}, { 0,  0}, { 2,  0}, { 4,  0}, { 6,  0},
         {-6,  2}, {-4,  2}, {-2,  2}, { 0,  2}, { 2,  2}, { 4,  2}, { 6,  2},
         {-6,  4}, {-4,  4}, {-2,  4}, { 0,  4}, { 2,  4}, { 4,  4}, { 6,  4},
         {-6,  6}, {-4,  6}, {-2,  6}, { 0,  6}, { 2,  6}, { 4,  6}, { 6,  6},
     }}
);

}

#endif //_SSVO_PATTERN_HPP_
