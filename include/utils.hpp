/**
 * @file utils.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 一些工具的声明
 * @version 0.1
 * @date 2019-01-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _SSVO_UTILS_HPP_
#define _SSVO_UTILS_HPP_

#include <opencv2/imgproc.hpp>
#include "global.hpp"

namespace ssvo {

/**
 * @brief 工具函数所在的命名空间
 * @details REVIEW 源代码没有看
 */
namespace utils {

/**
 * @brief 在给定的矩阵上的指定位置插入一个子矩阵
 * 
 * @tparam Ts           源矩阵的数据类型
 * @tparam Td           要插入的目标矩阵的数据类型 
 * @tparam Size         方阵的大小 TODO 
 * @param[in&out] src   源矩阵,这里是Eiegn形式的
 * @param[in] dst_ptr   要插入的目标矩阵
 * @param[in] dx_ptr    要插入的位置指针x
 * @param[in] dy_ptr    要插入的位置指针y
 * @param[in] u         TODO 
 * @param[in] v         TODO 
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const Matrix<Ts, Dynamic, Dynamic, RowMajor> &src, Td* dst_ptr, Td* dx_ptr, Td* dy_ptr, const double u, const double v)
{
    const int iu = floor(u);
    const int iv = floor(v);
    const float wu1 = u - iu;
    const float wu0 = 1.0 - wu1;
    const float wv1 = v - iv;
    const float wv0 = 1.0 - wv1;
    const float w_tl = wv0*wu0;
    const float w_tr = wv0*wu1;
    const float w_bl = wv1*wu0;
    const float w_br = 1.0f - w_tl - w_tr - w_bl;

    const int half_size = Size / 2;
    const int expand_size = Size + 2;
    const int expand_size1 = Size + 3;
    const int start_v = iv - half_size - 1;
    const int start_u = iu - half_size - 1;

    LOG_ASSERT(start_v >= 0 && start_u >= 0 && start_v+expand_size1 <= src.cols() && start_v+expand_size1 <= src.rows())
        << " Out of image scope! image cols=" << src.cols() << " rows=" << src.rows() << ", "
        << "LT: (" << start_u << ", " << start_v << ") - "
        << "BR: (" << start_u+expand_size1 << ", " << start_v+expand_size1 << ")";

    Matrix<Td, expand_size1, expand_size1, RowMajor>
        patch_with_border = src.block(start_v, start_u, expand_size1, expand_size1).template cast<Td>();
    //! block(i,j,p,q) i-rows j-cols
    Matrix<Td, expand_size, expand_size, RowMajor> mat_tl = w_tl * patch_with_border.block(0, 0, expand_size, expand_size);
    Matrix<Td, expand_size, expand_size, RowMajor> mat_tr = w_tr * patch_with_border.block(0, 1, expand_size, expand_size);
    Matrix<Td, expand_size, expand_size, RowMajor> mat_bl = w_bl * patch_with_border.block(1, 0, expand_size, expand_size);
    Matrix<Td, expand_size, expand_size, RowMajor> mat_br = w_br * patch_with_border.block(1, 1, expand_size, expand_size);

    Matrix<Td, expand_size, expand_size, RowMajor> mat_interpolate = mat_tl + mat_tr + mat_bl + mat_br;
    Matrix<Td, Size, Size, RowMajor> expand_img_x = mat_interpolate.block(1, 0, Size, Size);
    Matrix<Td, Size, Size, RowMajor> expand_img_y = mat_interpolate.block(0, 1, Size, Size);

    Eigen::Map<Matrix<Td, Size, Size, RowMajor> > dst(dst_ptr);
    Eigen::Map<Matrix<Td, Size, Size, RowMajor> > dx(dx_ptr);
    Eigen::Map<Matrix<Td, Size, Size, RowMajor> > dy(dy_ptr);
    dst = mat_interpolate.block(1, 1, Size, Size);
    dx = (mat_interpolate.block(1, 2, Size, Size) - expand_img_x) * 0.5;
    dy = (mat_interpolate.block(2, 1, Size, Size) - expand_img_y) * 0.5;
}

/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵的数据类型
 * @tparam Td           目标矩阵的数据类型
 * @tparam Size         大小 TODO 
 * @param[in&out] src   源矩阵,这里是cv::Mat形式的    
 * @param[in] dst_ptr   要插入的矩阵
 * @param[in] dx_ptr    插入位置的x指针
 * @param[in] dy_ptr    插入位置的y指针
 * @param[in] u         TODO 
 * @param[in] v         TODO 
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const cv::Mat &src, Td* dst_ptr, Td* dx_ptr, Td* dy_ptr, const double u, const double v)
{
    assert(src.type() == cv::DataType<Ts>::type);
    Eigen::Map<Matrix<Ts, Dynamic, Dynamic, RowMajor> > src_map(src.data, src.rows, src.cols);
    const int iu = floor(u);
    const int iv = floor(v);
    const float wu1 = u - iu;
    const float wu0 = 1.0 - wu1;
    const float wv1 = v - iv;
    const float wv0 = 1.0 - wv1;
    const float w_tl = wv0*wu0;
    const float w_tr = wv0*wu1;
    const float w_bl = wv1*wu0;
    const float w_br = 1.0f - w_tl - w_tr - w_bl;

    const int half_size = Size / 2;
    const int expand_size = Size + 2;
    const int expand_size1 = Size + 3;
    const int start_v = iv - half_size - 1;
    const int start_u = iu - half_size - 1;

    LOG_ASSERT(start_v >= 0 && start_u >= 0 && start_v+expand_size1 <= src.cols && start_v+expand_size1 <= src.rows)
    << " Out of image scope! image cols=" << src.cols << " rows=" << src.rows << ", "
    << "LT: (" << start_u << ", " << start_v << ") - "
    << "BR: (" << start_u+expand_size1 << ", " << start_v+expand_size1 << ")";

    Matrix<Td, expand_size1, expand_size1, RowMajor>
        patch_with_border = src_map.block(start_v, start_u, expand_size1, expand_size1).template cast<Td>();
    //! block(i,j,p,q) i-rows j-cols
    Matrix<Td, expand_size, expand_size, RowMajor> mat_tl = w_tl * patch_with_border.block(0, 0, expand_size, expand_size);
    Matrix<Td, expand_size, expand_size, RowMajor> mat_tr = w_tr * patch_with_border.block(0, 1, expand_size, expand_size);
    Matrix<Td, expand_size, expand_size, RowMajor> mat_bl = w_bl * patch_with_border.block(1, 0, expand_size, expand_size);
    Matrix<Td, expand_size, expand_size, RowMajor> mat_br = w_br * patch_with_border.block(1, 1, expand_size, expand_size);

    Matrix<Td, expand_size, expand_size, RowMajor> mat_interpolate = mat_tl + mat_tr + mat_bl + mat_br;
    Matrix<Td, Size, Size, RowMajor> expand_img_x = mat_interpolate.block(1, 0, Size, Size);
    Matrix<Td, Size, Size, RowMajor> expand_img_y = mat_interpolate.block(0, 1, Size, Size);

    Eigen::Map<Matrix<Td, Size, Size, RowMajor> > dst(dst_ptr);
    Eigen::Map<Matrix<Td, Size, Size, RowMajor> > dx(dx_ptr);
    Eigen::Map<Matrix<Td, Size, Size, RowMajor> > dy(dy_ptr);
    dst = mat_interpolate.block(1, 1, Size, Size);
    dx = (mat_interpolate.block(1, 2, Size, Size) - expand_img_x) * 0.5;
    dy = (mat_interpolate.block(2, 1, Size, Size) - expand_img_y) * 0.5;
}

/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵数据类型
 * @tparam Td           目标矩阵数据类型
 * @tparam Size         大小  TODO 
 * @param[in] src       源矩阵
 * @param[in] dst_ptr   目标矩阵
 * @param[in] u         TODO
 * @param[in] v         TODO 
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const Matrix<Ts, Dynamic, Dynamic, RowMajor> &src, Td* dst_ptr, const double u, const double v)
{
    const int iu = floor(u);
    const int iv = floor(v);
    const float wu1 = u - iu;
    const float wu0 = 1.0 - wu1;
    const float wv1 = v - iv;
    const float wv0 = 1.0 - wv1;
    const float w_tl = wv0*wu0;
    const float w_tr = wv0*wu1;
    const float w_bl = wv1*wu0;
    const float w_br = 1.0f - w_tl - w_tr - w_bl;

    const int half_size = Size / 2;
    const int expand_size = Size + 1;
    const int start_v = iv - half_size;
    const int start_u = iu - half_size;

    LOG_ASSERT(start_v >= 0 && start_u >= 0 && start_v+expand_size <= src.cols() && start_v+expand_size <= src.rows())
    << " Out of image scope! image cols=" << src.cols() << " rows=" << src.rows() << ", "
    << "LT: (" << start_u << ", " << start_v << ") - "
    << "BR: (" << start_u+expand_size << ", " << start_v+expand_size << ")";

    Matrix<Td, expand_size, expand_size, RowMajor> patch = src.block(start_v, start_u, expand_size, expand_size).template cast<Td>();
    Matrix<Td, Size, Size, RowMajor> mat_tl = w_tl * patch.block(0, 0, Size, Size);
    Matrix<Td, Size, Size, RowMajor> mat_tr = w_tr * patch.block(0, 1, Size, Size);
    Matrix<Td, Size, Size, RowMajor> mat_bl = w_bl * patch.block(1, 0, Size, Size);
    Matrix<Td, Size, Size, RowMajor> mat_br = w_br * patch.block(1, 1, Size, Size);

    Eigen::Map<Matrix<Td, Size, Size, RowMajor> > dst(dst_ptr);
    dst = mat_tl + mat_tr + mat_bl + mat_br;
}

/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵的数据类型
 * @tparam Td           目标矩阵的数据类型 
 * @tparam Size         大小 TODO
 * @param[in] src       源矩阵
 * @param[in] dst_ptr   要插入的目标矩阵
 * @param[in] u         TODO 
 * @param[in] v         TODO 
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const cv::Mat &src, Td* dst_ptr, const double u, const double v)
{
    assert(src.type() == cv::DataType<Ts>::type);
    Eigen::Map<Matrix<Ts, Dynamic, Dynamic, RowMajor> > src_map(src.data, src.rows, src.cols);
    const int iu = floor(u);
    const int iv = floor(v);
    const float wu1 = u - iu;
    const float wu0 = 1.0 - wu1;
    const float wv1 = v - iv;
    const float wv0 = 1.0 - wv1;
    const float w_tl = wv0*wu0;
    const float w_tr = wv0*wu1;
    const float w_bl = wv1*wu0;
    const float w_br = 1.0f - w_tl - w_tr - w_bl;

    const int half_size = Size / 2;
    const int expand_size = Size + 1;
    const int start_v = iv - half_size;
    const int start_u = iu - half_size;

    LOG_ASSERT(start_v >= 0 && start_u >= 0 && start_v+expand_size <= src.cols && start_v+expand_size <= src.rows)
    << " Out of image scope! image cols=" << src.cols << " rows=" << src.rows << ", "
    << "LT: (" << start_u << ", " << start_v << ") - "
    << "BR: (" << start_u+expand_size << ", " << start_v+expand_size << ")";

    Matrix<Td, expand_size, expand_size, RowMajor> patch = src_map.block(start_v, start_u, expand_size, expand_size).template cast<Td>();
    Matrix<Td, Size, Size, RowMajor> mat_tl = w_tl * patch.block(0, 0, Size, Size);
    Matrix<Td, Size, Size, RowMajor> mat_tr = w_tr * patch.block(0, 1, Size, Size);
    Matrix<Td, Size, Size, RowMajor> mat_bl = w_bl * patch.block(1, 0, Size, Size);
    Matrix<Td, Size, Size, RowMajor> mat_br = w_br * patch.block(1, 1, Size, Size);

    Eigen::Map<Matrix<Td, Size, Size, RowMajor> > dst(dst_ptr);
    dst = mat_tl + mat_tr + mat_bl + mat_br;
}

//! Eigen::Matrix
/**
 * @brief 插入矩阵
 * 
 * @tparam Ts       源矩阵数据类型
 * @tparam Td       目标矩阵数据类型
 * @tparam Size     TODO 大小
 * @param[in] src   源矩阵
 * @param[in] img   图像,也就是要插入的矩阵
 * @param[in] dx    TODO 看来前面的注释是错误的
 * @param[in] dy    TODO 
 * @param[in] u     TODO 可能这个才是真正插入的位置
 * @param[in] v     TODO 
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const Matrix<Ts, Dynamic, Dynamic, RowMajor> &src,
                           Matrix<Td, Size, Size, RowMajor> &img,
                           Matrix<Td, Size, Size, RowMajor> &dx,
                           Matrix<Td, Size, Size, RowMajor> &dy,
                           const double u, const double v)
{
    interpolateMat<Ts, Td, Size>(src, img.data(), dx.data(), dy.data(), u, v);
}

/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵数据类型
 * @tparam Td           目标矩阵数据类型
 * @tparam Size         TODO 大小
 * @param[in] src       目标矩阵
 * @param[in] img_vec   要插入的图像
 * @param[in] dx_vec    TODO
 * @param[in] dy_vec    TODO
 * @param[in] u         TODO 
 * @param[in] v         TODO 
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const Matrix<Ts, Dynamic, Dynamic, RowMajor> &src,
                           Matrix<Td, Size * Size, 1> &img_vec,
                           Matrix<Td, Size * Size, 1> &dx_vec,
                           Matrix<Td, Size * Size, 1> &dy_vec,
                           const double u, const double v)
{
    interpolateMat<Ts, Td, Size>(src, img_vec.data(), dx_vec.data(), dy_vec.data(), u, v);
}

/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵数据类型
 * @tparam Td           目标矩阵的数据类型
 * @tparam Size         大小 TODO 
 * @param[in] src       原矩阵
 * @param[in] img       TODO
 * @param[in] u         TODO
 * @param[in] v         TODO
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const Matrix<Ts, Dynamic, Dynamic, RowMajor> &src,
                           Matrix<Td, Size, Size, RowMajor> &img,
                           const double u, const double v)
{
    interpolateMat<Ts, Td, Size>(src, img.data(), u, v);
}

/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵数据类型
 * @tparam Td           目标矩阵的数据类型
 * @tparam Size         大小 TODO 
 * @param[in] src       原矩阵
 * @param[in] img_vec   TODO
 * @param[in] u         TODO
 * @param[in] v         TODO
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const Matrix<Ts, Dynamic, Dynamic, RowMajor> &src,
                           Matrix<Td, Size * Size, 1> &img_vec,
                           const double u, const double v)
{
    interpolateMat<Ts, Td, Size>(src, img_vec.data(), u, v);
}

//！ cv::Mat
/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵数据类型
 * @tparam Td           目标矩阵的数据类型
 * @tparam Size         大小 TODO 
 * @param[in] src       原矩阵
 * @param[in] img       要插入的图像?  TODO
 * @param[in] dx        TODO
 * @param[in] dy        TODO
 * @param[in] u         TODO
 * @param[in] v         TODO
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const cv::Mat &src,
                           Matrix<Td, Size, Size, RowMajor> &img,
                           Matrix<Td, Size, Size, RowMajor> &dx,
                           Matrix<Td, Size, Size, RowMajor> &dy,
                           const double u, const double v)
{
    interpolateMat<Ts, Td, Size>(src, img.data(), dx.data(), dy.data(), u, v);
}

/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵数据类型
 * @tparam Td           目标矩阵的数据类型
 * @tparam Size         大小 TODO 
 * @param[in] src       原矩阵
 * @param[in] img_vec   TODO
 * @param[in] dx_vec    TODO
 * @param[in] dy_vec    TODO
 * @param[in] u         TODO
 * @param[in] v         TODO
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const cv::Mat &src,
                           Matrix<Td, Size * Size, 1> &img_vec,
                           Matrix<Td, Size * Size, 1> &dx_vec,
                           Matrix<Td, Size * Size, 1> &dy_vec,
                           const double u, const double v)
{
    interpolateMat<Ts, Td, Size>(src, img_vec.data(), dx_vec.data(), dy_vec.data(), u, v);
}

/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵数据类型
 * @tparam Td           目标矩阵的数据类型
 * @tparam Size         大小 TODO 
 * @param[in] src       原矩阵
 * @param[in] img_vec   TODO
 * @param[in] u         TODO
 * @param[in] v         TODO
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const cv::Mat &src,
                           Matrix<Td, Size * Size, 1> &img_vec,
                           const double u, const double v)
{
    interpolateMat<Ts, Td, Size>(src, img_vec.data(), u, v);
}

/**
 * @brief 插入矩阵
 * 
 * @tparam Ts           源矩阵数据类型
 * @tparam Td           目标矩阵的数据类型
 * @tparam Size         大小 TODO 
 * @param[in] src       原矩阵
 * @param[in] u         TODO
 * @param[in] v         TODO
 */
template<typename Ts, typename Td, int Size>
inline void interpolateMat(const cv::Mat &src,
                           Matrix<Td, Size, Size, RowMajor> &img,
                           const double u, const double v)
{
    interpolateMat<Ts, Td, Size>(src, img.data(), u, v);
}

//! https://github.com/uzh-rpg/rpg_vikit/blob/master/vikit_common/include/vikit/vision.h
//! WARNING This function does not check whether the x/y is within the border
/**
 * @brief 插入矩阵
 * 
 * @tparam Ts       源矩阵的数据类型
 * @tparam Td       目标矩阵的数据类型
 * @param[in] mat   源矩阵? TODO
 * @param[in] u     TODO
 * @param[in] v     TODO
 * @return Td       插入矩阵后的结果
 */
template <typename Ts, typename Td>
inline Td interpolateMat(const cv::Mat& mat, const double u, const double v)
{
    assert(mat.type() == cv::DataType<Ts>::type);
    int x = floor(u);
    int y = floor(v);
    float wx1 = u - x;
    float wx0 = 1.0 - wx1;
    float wy1 = v - y;
    float wy0 = 1.0 - wy1;

    const int stride = mat.step[0]/mat.step[1];
    const Ts* ptr = mat.ptr<Ts>(y) + x;
    return (wx0*wy0)*ptr[0] + (wx1*wy0)*ptr[1] + (wx0*wy1)*ptr[stride] + (wx1*wy1)*ptr[stride + 1];
}

//! ===========================================================================================

/**
 * @brief 获取中位数
 * 
 * @tparam T            数据的类型
 * @param[in] data_vec  数据向量
 * @return T            得到的中位数
 */
template<class T>
inline T getMedian(std::vector<T> &data_vec)
{
    assert(!data_vec.empty());
    typename std::vector<T>::iterator it = data_vec.begin()+floor(data_vec.size()/2);
    std::nth_element(data_vec.begin(), it, data_vec.end());
    return *it;
}

/**
 * @brief 计算均匀分布概率
 * 
 * @tparam T            样本的数据类型
 * @param[in] x         样本
 * @param[in] mu        均值
 * @param[in] sigma     方差
 * @return double       概率
 */
template <typename T>
inline double normal_distribution(T x, T mu, T sigma)
{
    static const double inv_sqrt_2pi = 0.3989422804014327f;
    double a = (x - mu) / sigma;

    return inv_sqrt_2pi / sigma * std::exp(-0.5 * a * a);
}

/**
 * @brief TODO 减少啥啊
 * 
 * @tparam T            向量元素的数据类型
 * @param[in&out] vecs  向量1
 * @param[in] inliers   内点
 */
template <typename T>
inline void reduceVecor(std::vector<T>& vecs, const std::vector<bool>& inliers)
{
    size_t size = inliers.size();
    assert(size == vecs.size());

    typename std::vector<T>::iterator vecs_iter = vecs.begin();
    size_t idx = 0;
    for(;vecs_iter!=vecs.end();)
    {
        if(!inliers[idx])
        {
            inliers[idx] = inliers[--size];
            *vecs_iter = vecs.back();
            vecs.pop_back();
            continue;
        }
        idx++;
        vecs_iter++;
    }
}

/**
 * @brief 计算重投影误差??? TODO 前面不是已经有了专门的类了吗?
 * 
 * @param[in] fn    特征点的序列
 * @param[in] Tcw   相机的位姿变换
 * @param[in] pw    世界坐标系下这些点的3D坐标
 * @return double 
 */
inline double reprojectError(const Vector2d &fn, const SE3d &Tcw, const Vector3d &pw)
{
    Vector3d xyz_cur(Tcw*pw);
    Vector2d resdual = fn - xyz_cur.head<2>()/xyz_cur[2];
    return resdual.squaredNorm();
}

//! functions not using  template
/**
 * @brief Klt光流追踪
 * 
 * @param[in] imgs_ref          参考帧图像
 * @param[in] imgs_cur          当前帧图像
 * @param[in] win_size          窗口大小
 * @param[in] pts_ref           参考帧上的像素点
 * @param[in] pts_cur           当前帧上的像素点
 * @param[in] status            每个点的状态
 * @param[in] termcrit          TODO 貌似是什么准则
 * @param[in] track_forward     是否向前追踪,默认为否
 * @param[in] verbose           是否产生详情
 */
void kltTrack(const ImgPyr& imgs_ref, const ImgPyr& imgs_cur, const cv::Size win_size,
              const std::vector<cv::Point2f>& pts_ref, std::vector<cv::Point2f>& pts_cur,
              std::vector<bool> &status, cv::TermCriteria termcrit, bool track_forward = false, bool verbose = false);
/**
 * @brief 三角化点
 * 
 * @param[in] R_cr  从参考帧到当前帧的旋转 
 * @param[in] t_cr  从参考帧到当前帧的平移
 * @param[in] fn_r  参考帧的特征向量
 * @param[in] fn_c  当前帧的特征向量
 * @param[out] d_ref TODO ??? 三角化产生的点?
 * @return true 
 * @return false 
 */
bool triangulate(const Matrix3d &R_cr,  const Vector3d &t_cr, const Vector3d &fn_r, const Vector3d &fn_c, double &d_ref);

/**
 * @brief 和基础矩阵相关的工具函数
 * 
 */
namespace Fundamental
{

/**
 * @brief 计算基础矩阵
 * 
 * @param[in] pts_prev 前一帧中的点
 * @param[in] pts_next 当前帧中的点
 * @param[out] F       计算得到的基础矩阵             
 * @param[out] inliers  内点标记
 * @param[in] sigma2   TODO ???
 * @param[in] max_iterations 最大迭代次数,默认1000
 * @param[in] bE        TODO 
 * @return true 
 * @return false 
 */
bool findFundamentalMat(const std::vector<cv::Point2d> &pts_prev, const std::vector<cv::Point2d> &pts_next,
                       Matrix3d &F, std::vector<bool> &inliers,
                       const double sigma2 = 1, const int max_iterations = 1000, const bool bE = false);
/**
 * @brief 计算误差
 * @detials 详细地来说,应该是计算极线投影误差吗? TODO
 * @param[in] p1    平面点1    
 * @param[in] p2    平面点2
 * @param[in] F     计算得到的基础矩阵F
 * @param[in] err1  误差1,猜测可能是正向投影反向投影各能够获得一个投影的误差 TODO
 * @param[in] err2  误差2
 */
void computeErrors(const cv::Point2d &p1, const cv::Point2d &p2, Matrix3d &F, double &err1, double &err2);
/**
 * @brief 计算投影误差的平方??? TODO
 * 
 * @param[in] p1    空间点1
 * @param[in] p2    空间点2
 * @param[in] T     相机的变换矩阵
 * @param[in] p     平面点
 * @return double   得到的误差的平方
 */
double computeErrorSquared(const Vector3d &p1, const Vector3d &p2, const SE3d &T, const Vector2d &p);
/**
 * @brief 单位化
 * @detials 其实它的实现方法和ORB-SLAM2中的是类似的
 * @param[in] pts       平面点集合 
 * @param[out] pts_norm  归一化后的点的集合
 * @param[out] T         归一化矩阵
 */
void Normalize(const std::vector<cv::Point2d>& pts, std::vector<cv::Point2d>& pts_norm, Matrix3d& T);
/**
 * @brief 8点法求解基础矩阵
 * 
 * @param[in] pts_prev 前一帧中的像素点集合
 * @param[in] pts_next 后一帧中的像素点集合
 * @param[out] F       计算得到的基础矩阵
 * @param[in] bE       TODO
 * @return true 
 * @return false 
 */
bool run8point(const std::vector<cv::Point2d>& pts_prev, const std::vector<cv::Point2d>& pts_next,
               Matrix3d& F, const bool bE = false);
/**
 * @brief 进行RANSAC算法剔除外点
 * 
 * @param[in] pts_prev          前一帧中像素点集合
 * @param[in] pts_next          后一帧中像素点集合
 * @param[out] F                计算得到的基础矩阵
 * @param[out] inliers          得到的内点集合
 * @param[in] sigma2            TODO ??
 * @param[in] max_iterations    RANSAC最大迭代次数
 * @param[in] bE                TODO?
 * @return true 
 * @return false 
 */
bool runRANSAC(const std::vector<cv::Point2d>& pts_prev, const std::vector<cv::Point2d>& pts_next,
                Matrix3d& F, std::vector<bool> &inliers,
                const double sigma2 = 1, const int max_iterations = 1000, const bool bE = false);
/**
 * @brief 本征矩阵的分解
 * 
 * @param[in] E     等待进行分解的本征矩阵    
 * @param[out] R1   得到的一种旋转的情况
 * @param[out] R2   得到的第二种旋转的情况
 * @param[out] t    得到的一种平移的情况,另外一种可以通过直接加符号来获得
 */
void decomposeEssentialMat(const Matrix3d& E, Matrix3d& R1, Matrix3d& R2, Vector3d& t);

}//! namespace Fundamental

}//! namespace utils

}//! namespace ssvo

#endif //_SSVO_UTILS_HPP_
