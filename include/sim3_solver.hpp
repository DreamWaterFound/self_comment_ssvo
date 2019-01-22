/**
 * @file sim3_solver.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 求解sim3
 * @version 0.1
 * @date 2019-01-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _SSVO_SIM3_SOLVER_HPP_
#define _SSVO_SIM3_SOLVER_HPP_

#include "global.hpp"
#include "feature.hpp"
#include "map_point.hpp"
#include "optimizer.hpp"

namespace ssvo{

/**
 * @brief Sim3解算器
 * 
 */
class Sim3Solver{
public:

    /**
     * @brief 构造函数
     * 
     * @param[in] Tcw1          帧1的变换
     * @param[in] Tcw2          帧2的变换
     * @param[in] fts1          帧1的特征
     * @param[in] fts2          帧2的特征
     * @param[in] scale_fixed   TODO 是否固定尺度?
     */
    Sim3Solver(const SE3d Tcw1, const SE3d Tcw2, const std::vector<Feature::Ptr> &fts1, const std::vector<Feature::Ptr> &fts2, const bool scale_fixed) :
        scale_fixed_(scale_fixed)
    {
        const size_t N = fts1.size();
        LOG_ASSERT(N == fts2.size()) << "fts1(" << N << ") != fts2(" << fts2.size() << ")!";

        mpts1_.reserve(N);
        mpts2_.reserve(N);
        pxls1_.reserve(N);
        pxls2_.reserve(N);
        max_err1_.reserve(N);
        max_err2_.reserve(N);
        indices_.reserve(N);

        const Matrix3d Rcw1 = Tcw1.rotationMatrix();
        const Matrix3d Rcw2 = Tcw2.rotationMatrix();
        const Vector3d tcw1 = Tcw1.translation();
        const Vector3d tcw2 = Tcw2.translation();

        for(size_t i = 0; i < N; i++)
        {
            const Feature::Ptr &ft1 = fts1[i];
            const Feature::Ptr &ft2 = fts2[i];

            if(ft1->mpt_ == nullptr || ft2->mpt_ == nullptr)
                continue;
            if(ft1->mpt_->isBad() || ft2->mpt_->isBad())
                continue;

            mpts1_.push_back(Rcw1 * ft1->mpt_->pose() + tcw1);
            mpts2_.push_back(Rcw2 * ft2->mpt_->pose() + tcw2);

            pxls1_.push_back(ft1->fn_.head<2>()/ft1->fn_[2]);
            pxls1_.push_back(ft2->fn_.head<2>()/ft2->fn_[2]);

            const double sigm_square1 = 1 << ft1->level_;
            const double sigm_square2 = 1 << ft2->level_;

            max_err1_.push_back(9.210 * sigm_square1);
            max_err2_.push_back(9.210 * sigm_square2);

            indices_.push_back(i);
        }

        N_ = N;
        noMore_ = false;
    }

    /**
     * @brief 部署RANSAC算法
     * 
     * @param[in] max_iterations    最大的迭代次数,默认为5
     * @return true 
     * @return false 
     */
    bool runRANSAC(const int max_iterations = 5);

    /**
     * @brief 获取估计得到的Sim3位姿
     * 
     * @param[out] R             旋转
     * @param[out] t             平移
     * @param[out] s             尺度
     * @param[out] inliers       内点个数
     */
    void getEstimateSim3(Matrix3d &R, Vector3d &t, double &s, std::vector<bool> &inliers);
    /**
     * @brief 计算sim3变换
     * 
     * @param[in] pts1          帧1中的点
     * @param[in] pts2          帧2中的点
     * @param[in] R12           帧2->帧1的旋转变换
     * @param[in] t12           帧2->帧1的平移变换
     * @param[in] s12           帧2->帧1的缩放
     * @param[in] scale_fixed   是否适应尺度? 
     */
    static void computeSim3(const std::vector<Vector3d> &pts1, const std::vector<Vector3d> &pts2, Matrix3d &R12, Vector3d &t12, double &s12, bool scale_fixed = false);
    /**
     * @brief 设置进行RANSAC算法所需要的参数
     * 
     * @param[in] probability   概率? TODO  默认为0.99 
     * @param[in] minInliers    最小的内点个数,默认为20
     * @param[in] maxIterations 最大的迭代次数,默认为300
     */
    void SetRansacParameters(double probability = 0.99, int minInliers = 20 , int maxIterations = 300);


private:

    /**
     * @brief 检查内点个数?  TODO
     *  
     * @param[in] R12       旋转
     * @param[in] t12       平移
     * @param[in] s12       缩放
     * @param[in] inliers   内点
     * @return int          检查后的内点个数? TODO 
     */
    int checkInliers(Matrix3d R12, Vector3d t12, double s12, std::vector<bool>& inliers);

private:

    ///是否锁定缩放? TODO 
    const bool scale_fixed_;

    ///TODO
    size_t N_;
    ///索引
    std::vector<size_t> indices_;
    ///帧1中的地图点
    std::vector<Vector3d> mpts1_;
    ///帧2中的地图点
    std::vector<Vector3d> mpts2_;
    ///帧1中的点(图像像素坐标)
    std::vector<Vector2d> pxls1_; //归一化平面中前两维
    ///帧2中的点(图像像素坐标)
    std::vector<Vector2d> pxls2_;

    ///帧1中的最大误差 TODO 为什么存储于vector中?
    std::vector<double> max_err1_;
    ///帧2中的最大误差 TODO 
    std::vector<double> max_err2_;

    ///存储内点标志
    std::vector<bool> inliers_;

    ///最佳的旋转
    Matrix3d R12_best_;
    ///最佳的平移
    Vector3d t12_best_;
    ///最佳的缩放系数
    double s12_best_;
    ///得到的最好情况下的内点个数(此时不一定是最多)
    int inliers_count_best_;

    ///RANSAC最大迭代次数
    int maxIterations_;
    ///概率??? TODO 
    double probability_;
    ///最小允许的内点个数
    int min_inliers_;
    ///设置的迭代次数
    int iterations_;

public:
    /// TODO 
    bool noMore_;

};//! Sim3Solver


}


#endif //_SSVO_SIM3_SOLVER_HPP_
