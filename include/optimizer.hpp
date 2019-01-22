/**
 * @file optimizer.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 优化器的实现
 * @version 0.1
 * @date 2019-01-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _OPTIMIZER_HPP_
#define _OPTIMIZER_HPP_

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "map_point.hpp"
#include "keyframe.hpp"
#include "map.hpp"
#include "global.hpp"
//TODO 为啥不包括进来呢?
//#include "loop_closure.hpp"

namespace ssvo {

//class LoopClosure;

//REVIEW 所有的实现代码都没有开始看

/**
 * @brief 优化器
 * @detials 这个优化器也是不可被复制的类型
 */
class Optimizer: public noncopyable
{
public:
    ///管理本类的指针
    typedef std::map<
                KeyFrame::Ptr,                                          //索引图中元素所使用的key
                Sophus::Sim3d ,                                         //在图中被索引的内容
                std::less<KeyFrame::Ptr>,                               //用来比较key相互之间大小的..判别器
                Eigen::aligned_allocator<std::pair<                     //TODO 
                                            const KeyFrame::Ptr, 
                                            Sophus::Sim3d>
                                        >
                    > KeyFrameAndPose;

    /**
     * @brief 进行全局BA
     * @detials 静态成员函数
     * @param[in] map           地图
     * @param[in] max_iters     最大迭代次数
     * @param[in] nLoopKF       TODO 检测到的回环关键帧?
     * @param[in] report        是否汇报
     * @param[in] verbose       是否产生详情
     * TODO 输出在哪里?
     */
    static void globleBundleAdjustment(const Map::Ptr &map, int max_iters,const uint64_t nLoopKF = 0, bool report=false, bool verbose=false);
    /**
     * @brief 仅仅是对相机的运动进行BA
     * @detials 静态成员函数
     * @param[in] frame         帧
     * @param[in] use_seeds     是否使用种子 TODO 
     * @param[in] reject        TODO 
     * @param[in] report        是否汇报
     * @param[in] verbose       是否产生详情
     */
    static void motionOnlyBundleAdjustment(const Frame::Ptr &frame, bool use_seeds, bool reject=false, bool report=false, bool verbose=false);
    /**
     * @brief 局部BA
     * 
     * @param[in] keyframe          关键帧
     * @param[TODO] bad_mpts        坏地图点列表
     * @param[in] size              TODO 什么的大小?
     * @param[in] min_shared_fts    TODO 
     * @param[in] report            是否汇报
     * @param[in] verbose           是否产生详情
     */
    static void localBundleAdjustment(const KeyFrame::Ptr &keyframe, std::list<MapPoint::Ptr> &bad_mpts, int size=10, int min_shared_fts=50, bool report=false, bool verbose=false);
    /**
     * @brief 优化sim3位姿
     * 
     * @param[in] pKF1              关键帧1
     * @param[in] pKF2              关键帧2
     * @param[in] vpMatches1        TODO 匹配关系?
     * @param[out] S12              Sim_2to1
     * @param[in] th2               TODO 
     * @param[in] bFixScale         TODO         
     * @return int                  TODO 
     */
    static int optimizeSim3(KeyFrame::Ptr pKF1, KeyFrame::Ptr pKF2, std::vector<MapPoint::Ptr> &vpMatches1,
                                  Sophus::Sim3d &S12, const float th2, const bool bFixScale);

    /**
     * @brief 优化EssentialGraph
     * @detials 参数的输入输出属性目前暂不明确
     * @param[in] pMap                  地图
     * @param[in] pLoopKF               产生回环的关键帧 ?  TODO 
     * @param[in] pCurKF                当前的关键帧? TODO 
     * @param[in] NonCorrectedSim3      没有被矫正的Sim3
     * @param[out] CorrectedSim3         矫正后的Sim3变换
     * @param[in] LoopConnections       回环检测到的边链接? TODO  
     * @param[in] bFixScale             TODO ??
     */
    static void OptimizeEssentialGraph(Map::Ptr pMap, KeyFrame::Ptr pLoopKF, KeyFrame::Ptr pCurKF, KeyFrameAndPose &NonCorrectedSim3, KeyFrameAndPose &CorrectedSim3,
                                             const std::map<KeyFrame::Ptr, std::set<KeyFrame::Ptr> > &LoopConnections, const bool &bFixScale = false);

//    static void localBundleAdjustmentWithInvDepth(const KeyFrame::Ptr &keyframe, std::list<MapPoint::Ptr> &bad_mpts, int size=10, bool report=false, bool verbose=false);

    /**
     * @brief 优化地图点
     * 
     * @param[in] mpt       给定的地图点 
     * @param[in] max_iter  最大迭代次数
     * @param[in] report    是否汇报
     * @param[in] verbose   是否产生详情
     */
    static void refineMapPoint(const MapPoint::Ptr &mpt, int max_iter, bool report=false, bool verbose=false);

    /**
     * @brief 计算产生评价用的残差
     * 
     * @tparam nRes                                 残差的个数
     * @param[in] problem                           优化问题的问题句柄
     * @param[in] id                                TODO ???
     * @return Eigen::Matrix<double, nRes, 1>       TODO 返回得到的残差矩阵,nRes x 1 的矩阵
     */
    template<int nRes>
    static inline Eigen::Matrix<double, nRes, 1> evaluateResidual(const ceres::Problem& problem, ceres::ResidualBlockId id)
    {
        auto cost = problem.GetCostFunctionForResidualBlock(id);
        std::vector<double*> parameterBlocks;
        problem.GetParameterBlocksForResidualBlock(id, &parameterBlocks);
        Eigen::Matrix<double, nRes, 1> residual;
        cost->Evaluate(parameterBlocks.data(), residual.data(), nullptr);
        return residual;
    }

    /**
     * @brief 报告信息
     * 
     * @tparam nRes             残差的个数
     * @param[in] problem       优化问题句柄
     * @param[in] summary       TODO ???
     * @param[in] report        是否汇报
     * @param[in] verbose       是否产生详情
     */
    template<int nRes>
    static inline void reportInfo(const ceres::Problem &problem, const ceres::Solver::Summary summary, bool report = false, bool verbose = false)
    {
        if (!report) return;

        if (!verbose)
        {
            LOG(INFO) << summary.BriefReport();
        }
        else
        {
            LOG(INFO) << summary.FullReport();
            std::vector<ceres::ResidualBlockId> ids;
            problem.GetResidualBlocks(&ids);
            for (size_t i = 0; i < ids.size(); ++i)
            {
                LOG(INFO) << "BlockId: " << std::setw(5) << i << " residual(RMSE): " << evaluateResidual<nRes>(problem, ids[i]).norm();
            }
        }
    }
};

/**
 * @brief ceres优化器使用的名字空间
 * @see https://github.com/strasdat/Sophus/blob/v1.0.0/test/ceres/local_parameterization_se3.hpp
 */
namespace ceres_slover {
// https://github.com/strasdat/Sophus/blob/v1.0.0/test/ceres/local_parameterization_se3.hpp

/**
 * @brief 对SE3位姿进行参数优化的类
 * 
 */
class SE3Parameterization : public ceres::LocalParameterization {
public:

    /**
     * @brief 析构函数
     * @detials 由子类负责进行实现
     */
    virtual ~SE3Parameterization() {}

    /**
     * @brief 加法操作
     * 
     * @param[in] T_raw             位姿
     * @param[in] delta_raw         扰动量
     * @param[out] T_plus_delta_raw  位姿+扰动量
     * @return true                 计算结果恒true
     * @return false                不存在的
     */
    virtual bool Plus(double const *T_raw, double const *delta_raw,
                      double *T_plus_delta_raw) const {
        Eigen::Map<Sophus::SE3d const> const T(T_raw);
        Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
        Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
        T_plus_delta = Sophus::SE3d::exp(delta) * T;
        return true;
    }

    // Set to Identity, for we have computed in ReprojectionErrorSE3::Evaluate
    /**
     * @brief 计算雅克比
     * @detials Set to Identity, for we have computed in ReprojectionErrorSE3::Evaluate
     * \n 但是在这里并没有这样子设计,而是将雅克比设置成为了单位变换的形式,这是因为我们将会在 ReprojectionErrorSE3::Evaluate() 中进行计算
     * @param[in] T_raw         位姿
     * @param[out] jacobian_raw  得到的雅克比
     * @return true 
     * @return false 
     */
    virtual bool ComputeJacobian(double const *T_raw,
                                 double *jacobian_raw) const {
        Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_raw);
        jacobian.block<6,6>(0, 0).setIdentity();
        jacobian.rightCols<1>().setZero();
        return true;
    }

    /**
     * @brief 全局参数优化块大小?  TODO
     * 
     * @return int 大小
     */
    virtual int GlobalSize() const { return Sophus::SE3d::num_parameters; }
    /**
     * @brief 局部参数优化块大小? TODO 
     * 
     * @return int 大小
     */
    virtual int LocalSize() const { return Sophus::SE3d::DoF; }
};

/**
 * @brief 存储重投影误差的结构体
 * 
 */
struct ReprojectionError {
    /**
     * @brief 构造函数
     * 
     * @param[in] observed_x    某个点的观测值的x坐标
     * @param[in] observed_y    某个点的观测值的y坐标
     */
    ReprojectionError(double observed_x, double observed_y)
        : observed_x_(observed_x), observed_y_(observed_y) {}

    /**
     * @brief 重载括号运算符
     * 
     * @tparam T                TODO 
     * @param[in] camera        相机模型
     * @param[in] point         地图点
     * @param[out] residuals     误差
     * @return true 
     * @return false 
     */
    template<typename T>
    bool operator()(const T *const camera, const T *const point, T *residuals) const {
        Sophus::SE3<T> pose = Eigen::Map<const Sophus::SE3<T> >(camera);
        Eigen::Matrix<T, 3, 1> p = Eigen::Map<const Eigen::Matrix<T, 3, 1> >(point);

        Eigen::Matrix<T, 3, 1> p1 = pose.rotationMatrix() * p + pose.translation();

        T predicted_x = (T) p1[0] / p1[2];
        T predicted_y = (T) p1[1] / p1[2];
        residuals[0] = predicted_x - T(observed_x_);
        residuals[1] = predicted_y - T(observed_y_);
        return true;
    }

    /**
     * @brief 创建重投影误差
     * 
     * @param[in] observed_x            观测值x 
     * @param[in] observed_y            观测值y
     * @return ceres::CostFunction*     计算得到的损失函数的指针
     */
    static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, Sophus::SE3d::num_parameters, 3>(
            new ReprojectionError(observed_x, observed_y)));
    }

    ///观测值x坐标
    double observed_x_;
    ///观测值y坐标
    double observed_y_;
};

/**
 * @brief 定义SE3上的重投影误差计算类
 * @details 本质上还是损失函数
 */
class ReprojectionErrorSE3 : public ceres::SizedCostFunction<2, 7, 3>
{
public:
    /**
     * @brief 构造函数
     * 
     * @param[in] observed_x 观测值x坐标
     * @param[in] observed_y 观测值y坐标
     * @param[in] weight     权重
     */
    ReprojectionErrorSE3(double observed_x, double observed_y, double weight)
        : observed_x_(observed_x), observed_y_(observed_y), weight_(weight) {}
    /**
     * @brief 评价??? 评价什么? 得到权重吗? TODO 
     * 
     * @param[in] parameters    参数,这里是参数q,t  TODO 是sim3中的参数吗?
     * @param[TODO] residuals     残差
     * @param[TODO] jacobians     雅克比
     * @return true 
     * @return false 
     */
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        //! In Sophus, stored in the form of [q, t]
        Eigen::Map<const Eigen::Quaterniond> q(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> t(parameters[0] + 4);
        Eigen::Map<const Eigen::Vector3d> p(parameters[1]);

        Eigen::Vector3d p1 = q * p + t;

        const double predicted_x =  p1[0] / p1[2];
        const double predicted_y =  p1[1] / p1[2];
        residuals[0] = predicted_x - observed_x_;
        residuals[1] = predicted_y - observed_y_;

        residuals[0] *= weight_;
        residuals[1] *= weight_;

        if(!jacobians) return true;
        double* jacobian0 = jacobians[0];
        double* jacobian1 = jacobians[1];

        //! The point observed is in the normalized plane
        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> jacobian;

        const double z_inv = 1.0 / p1[2];
        const double z_inv2 = z_inv*z_inv;
        jacobian << z_inv, 0.0, -p1[0]*z_inv2,
                    0.0, z_inv, -p1[1]*z_inv2;

        jacobian.array() *= weight_;

        if(jacobian0 != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > Jse3(jacobian0);
            Jse3.setZero();
            //! In the order of Sophus::Tangent
            Jse3.block<2,3>(0,0) = jacobian;
            Jse3.block<2,3>(0,3) = jacobian*Sophus::SO3d::hat(-p1);
        }
        if(jacobian1 != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jpoint(jacobian1);
            Jpoint = jacobian * q.toRotationMatrix();
        }
        return true;
    }
    /**
     * @brief 创建一个重投影误差类
     * 
     * @param[in] observed_x            观测值x坐标
     * @param[in] observed_y            观测值y坐标
     * @param[in] weight                权重
     * @return ceres::CostFunction*     得到的损失函数的指针
     */
    static inline ceres::CostFunction *Create(const double observed_x, const double observed_y, const double weight = 1.0) {
        return (new ReprojectionErrorSE3(observed_x, observed_y, weight));
    }

private:
    ///观测值x坐标
    double observed_x_;
    ///观测值y坐标
    double observed_y_;
    ///观测值的权重 TODO 
    double weight_;

}; // class ReprojectionErrorSE3

/**
 * @brief 从逆深度计算SE3上的重投影误差
 * 
 */
class ReprojectionErrorSE3InvDepth : public ceres::SizedCostFunction<2, 7, 7, 1>
{
public:
    /**
     * @brief 构造函数
     * 
     * @param[in] observed_x_ref 参考帧上点像素的x坐标
     * @param[in] observed_y_ref 参考帧上点像素的y坐标
     * @param[in] observed_x_cur 当前帧上点像素的x坐标
     * @param[in] observed_y_cur 当前帧上点像素的y坐标
     */
    ReprojectionErrorSE3InvDepth(double observed_x_ref, double observed_y_ref, double observed_x_cur, double observed_y_cur)
        : observed_x_ref_(observed_x_ref), observed_y_ref_(observed_y_ref),
          observed_x_cur_(observed_x_cur), observed_y_cur_(observed_y_cur) {}
    /**
     * @brief 评估 TODO 
     * 
     * @param[in]  parameters  要评估的参数 TODO 都有啥啊,都是什意思啊
     * @param[out] residuals    评估得到的残差 
     * @param[out] jacobians    评估得到的雅克比
     * @return true 
     * @return false 
     */
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Map<const Sophus::SE3d> T_ref(parameters[0]);
        Eigen::Map<const Sophus::SE3d> T_cur(parameters[1]);
        const double inv_z = parameters[2][0];

        const Eigen::Vector3d p_ref(observed_x_ref_/inv_z, observed_y_ref_/inv_z, 1.0/inv_z);
        const Sophus::SE3d T_cur_ref = T_cur * T_ref.inverse();
        const Eigen::Vector3d p_cur = T_cur_ref * p_ref;

        const double predicted_x =  p_cur[0] / p_cur[2];
        const double predicted_y =  p_cur[1] / p_cur[2];
        residuals[0] = predicted_x - observed_x_cur_;
        residuals[1] = predicted_y - observed_y_cur_;

        if(!jacobians) return true;
        double* jacobian0 = jacobians[0];
        double* jacobian1 = jacobians[1];
        double* jacobian2 = jacobians[2];

        //! The point observed is in the normalized plane
        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> Jproj;

        const double z_inv = 1.0 / p_cur[2];
        const double z_inv2 = z_inv*z_inv;
        Jproj << z_inv, 0.0, -p_cur[0]*z_inv2,
            0.0, z_inv, -p_cur[1]*z_inv2;

        if(jacobian0 != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > JRse3(jacobian0);
            JRse3.setZero();
            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> JRP = Jproj*T_cur_ref.rotationMatrix();
            JRse3.block<2,3>(0,0) = -JRP;
            JRse3.block<2,3>(0,3) = JRP*Sophus::SO3d::hat(p_ref);
        }
        if(jacobian1 != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > JCse3(jacobian1);
            JCse3.setZero();
            JCse3.block<2,3>(0,0) = Jproj;
            JCse3.block<2,3>(0,3) = Jproj*Sophus::SO3d::hat(-p_cur);
        }
        if(jacobian2 != nullptr)
        {
//            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jp(jacobian2);
//            Eigen::Matrix3d Jpp(T_cur_ref.rotationMatrix());
//            Jpp.col(2) = T_cur_ref.rotationMatrix() * (-p_ref);
//            Jp.noalias() = Jproj * Jpp * p_ref[2];
            Eigen::Map<Eigen::RowVector2d> Jp(jacobian2);
            Jp = Jproj * T_cur_ref.rotationMatrix() * p_ref * (-1.0/inv_z);
        }
        return true;
    }
    /**
     * @brief 创建当前这个类的对象
     * 
     * @param[in] observed_x_ref 参考帧上点像素的x坐标
     * @param[in] observed_y_ref 参考帧上点像素的y坐标
     * @param[in] observed_x_cur 当前帧上点像素的x坐标
     * @param[in] observed_y_cur 当前帧上点像素的y坐标
     * @return ceres::CostFunction* 实例指针,但是这个类型是当前这个类的父类  TODO 上面的注释可能需要进行修改
     */
    static inline ceres::CostFunction *Create(double observed_x_ref, double observed_y_ref,
                                              double observed_x_cur, double observed_y_cur) {
        return (new ReprojectionErrorSE3InvDepth(observed_x_ref, observed_y_ref, observed_x_cur, observed_y_cur));
    }

private:
    ///参考帧上点像素的x坐标
    double observed_x_ref_;
    ///参考帧上点像素的y坐标
    double observed_y_ref_;
    ///当前帧上点像素的x坐标
    double observed_x_cur_;
    ///当前帧上点像素的y坐标
    double observed_y_cur_;

};
/**
 * @brief 点的逆,是什么意思? TODO 以及为什么要有这么多的和重投影相关的类
 * 
 */
class ReprojectionErrorSE3InvPoint : public ceres::SizedCostFunction<2, 7, 7, 3>
{
public:

    /**
     * @brief 构造函数
     * 
     * @param[in] observed_x  观测值的x坐标
     * @param[in] observed_y  观测值的y坐标
     */
    ReprojectionErrorSE3InvPoint(double observed_x, double observed_y)
        : observed_x_(observed_x), observed_y_(observed_y){}

    /**
     * @brief 评估
     * 
     * @param[in] parameters  要进行评估的参数
     * @param[out] residuals  得到的残差
     * @param[out] jacobians  得到的雅克比
     * @return true 
     * @return false 
     */
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        Eigen::Map<const Sophus::SE3d> T_ref(parameters[0]);
        Eigen::Map<const Sophus::SE3d> T_cur(parameters[1]);
        Eigen::Map<const Eigen::Vector3d> inv_p(parameters[2]);
        Sophus::SE3d T_cur_ref = T_cur * T_ref.inverse();

        const Eigen::Vector3d p_ref(inv_p[0] / inv_p[2], inv_p[1] / inv_p[2], 1.0 / inv_p[2]);
        const Eigen::Vector3d p_cur = T_cur_ref * p_ref;

        const double predicted_x =  p_cur[0] / p_cur[2];
        const double predicted_y =  p_cur[1] / p_cur[2];
        residuals[0] = predicted_x - observed_x_;
        residuals[1] = predicted_y - observed_y_;

        if(!jacobians) return true;
        double* jacobian0 = jacobians[0];
        double* jacobian1 = jacobians[1];
        double* jacobian2 = jacobians[2];

        //! The point observed is in the normalized plane
        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> Jproj;

        const double z_inv = 1.0 / p_cur[2];
        const double z_inv2 = z_inv*z_inv;
        Jproj << z_inv, 0.0, -p_cur[0]*z_inv2,
            0.0, z_inv, -p_cur[1]*z_inv2;

        if(jacobian0 != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > JRse3(jacobian0);
            JRse3.setZero();
            Eigen::Matrix<double, 2, 3, Eigen::RowMajor> JRP = Jproj*T_cur_ref.rotationMatrix();
            JRse3.block<2,3>(0,0) = -JRP;
            JRse3.block<2,3>(0,3) = JRP*Sophus::SO3d::hat(p_ref);
        }
        if(jacobian1 != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor> > JCse3(jacobian1);
            JCse3.setZero();
            JCse3.block<2,3>(0,0) = Jproj;
            JCse3.block<2,3>(0,3) = Jproj*Sophus::SO3d::hat(-p_cur);
        }
        if(jacobian2 != nullptr)
        {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > Jp(jacobian2);
            Eigen::Matrix3d Jpp(T_cur_ref.rotationMatrix());
            Jpp.col(2) = T_cur_ref.rotationMatrix() * (-p_ref);
            Jp.noalias() = Jproj * Jpp * p_ref[2];
        }
        return true;
    }

    /**
     * @brief 创建当前类的实例对象
     * 
     * @param[in] observed_x    观测值x坐标
     * @param[in] observed_y    观测值y坐标
     * @return ceres::CostFunction*     实例指针
     */
    static inline ceres::CostFunction *Create(double observed_x, double observed_y) {
        return (new ReprojectionErrorSE3InvPoint(observed_x, observed_y));
    }

private:
    ///观测值x坐标
    double observed_x_;
    ///观测值y坐标
    double observed_y_;

};

//!**********************************************************************************************************************************************

/**
 * @brief TODO 好像是使用另外的一种方式来计算残差
 * 
 */
struct IntrinsicReprojErrorOnlyPose
{
    /**
     * @brief 构造函数
     * 
     * @param[in] observed_x    观测值的x坐标
     * @param[in] observed_y    观测值的y坐标
     * @param[in] pos           相机的位姿? TODO 
     * @param[in] level         特征点所在的图层
     * @param[in] pKF           相关的关键帧? TODO 
     */
    IntrinsicReprojErrorOnlyPose(double observed_x, double observed_y, double* pos,int level,KeyFrame::Ptr pKF)
    {
        obs_x_ = observed_x;
        obs_y_ = observed_y;
        Mp_Pos_ << pos[0], pos[1], pos[2];
        level_ = level;
        pKF_ = pKF;
    }

    /**
     * @brief 重载括号运算符
     * 
     * @param[in] camera    相机的内参? TODO
     * @param[in] residuals 计算得到的残差?
     * @return true 
     * @return false 
     */
    bool operator()(const double* const camera, double* residuals) const
    {
        Eigen::Quaterniond q = Eigen::Quaterniond(camera[3],camera[0],camera[1],camera[2]);
        Vector3d t = Vector3d(camera[4],camera[5],camera[6]);

        //todo 检查这里的问题
        if(q.squaredNorm()<0.001||q.squaredNorm()>1000)
            q.normalize();
//        std::cout<<"q.squaredNorm(): "<<q.squaredNorm()<<std::endl;

        Sophus::Sim3d Sim3_cam(q,t);
        Vector3d Mp_cam = Sim3_cam * Mp_Pos_;
        Vector2d px = pKF_->cam_->project(Mp_cam);

        residuals[0] = (obs_x_ - px[0])/sqrt(1<<level_);
        residuals[1] = (obs_y_ - px[1])/sqrt(1<<level_);
        return true;
    }

    ///观测值
    double obs_x_, obs_y_;
    ///地图点的位置
    Vector3d Mp_Pos_;
    ///对应特征所在的图层
    int level_;
    ///生成这个地图点所在的关键帧
    KeyFrame::Ptr pKF_;
};

/**
 * @brief ????? TODO 
 * 
 */
struct IntrinsicReprojErrorOnlyPoseInvSim3
{
    /**
     * @brief 构造函数
     * 
     * @param[in] observed_x 观测值的x坐标
     * @param[in] observed_y 观测值的y坐标
     * @param[in] pos        地图点的空间位置
     * @param[in] level      特征所在的图层
     * @param[in] pKF        生成相应地图点的关键帧
     */
    IntrinsicReprojErrorOnlyPoseInvSim3(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF)
    {
        obs_x_ = observed_x;
        obs_y_ = observed_y;
        Mp_Pos_<< pos[0], pos[1], pos[2];
        level_ = level;
        pKF_ =pKF;
    }

    /**
     * @brief 重载括号运算符
     * 
     * @param[in] camera        TODO 相机参数??
     * @param[out] residuals     计算得到的残差?  TODO
     * @return true 
     * @return false 
     */
    bool operator()(const double* const camera, double* residuals) const
    {

        Eigen::Quaterniond q = Eigen::Quaterniond(camera[3],camera[0],camera[1],camera[2]);
        Vector3d t = Vector3d(camera[4],camera[5],camera[6]);

        //TODO 检查这里的问题
        if(q.squaredNorm()<0.001||q.squaredNorm()>1000)
            q.normalize();
//        std::cout<<"q.squaredNorm(): "<<q.squaredNorm()<<std::endl;

        Sophus::Sim3d Sim3_k12(q,t);

        Vector3d Mp_cam = Mp_Pos_;
        Mp_cam = Sim3_k12.inverse() * Mp_cam;

        Vector2d px = pKF_->cam_->project(Mp_cam);

        //! add weigh
        residuals[0] = (px[0] - obs_x_)/sqrt(1<<level_);
        residuals[1] = (px[1] - obs_y_)/sqrt(1<<level_);

        return true;
    }

    ///观测值
    double obs_x_, obs_y_;
    ///地图点的空间位置
    Vector3d Mp_Pos_;
    ///特征所在的图层
    int level_;
    ///生成这个地图点的关键帧
    KeyFrame::Ptr pKF_;
};

/**
 * @brief 只根据位置计算得到的重投影误差
 * 
 */
struct ReprojErrorOnlyPose
{
    /**
     * @brief 构造函数
     * 
     * @param[in] observed_x 观测值的x坐标
     * @param[in] observed_y 观测值的y坐标
     * @param[in] pos        地图点的位置
     * @param[in] level      对应特征所在的图层
     * @param[in] pKF        生成这个地图点的关键帧
     */
    ReprojErrorOnlyPose(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF):
            intrinsicReprojErrorOnlyPose_(new ceres::NumericDiffCostFunction<IntrinsicReprojErrorOnlyPose,ceres::CENTRAL,2,7>(
            new IntrinsicReprojErrorOnlyPose(observed_x,observed_y,pos,level,pKF)))
    {}
    /**
     * @brief 重载括号运算符
     * 
     * @tparam T                TODO 
     * @param[in] camera        相机参数??? TODO 
     * @param[out] residuals     计算得到的残差
     * @return true 
     * @return false 
     */
    template <typename T> bool operator()(const T* const camera, T* residuals) const
    {
        return intrinsicReprojErrorOnlyPose_(camera,residuals);
    }
    /**
     * @brief 创建实例
     * 
     * @param[in] observed_x    观测值的x坐标
     * @param[in] observed_y    观测值的y坐标
     * @param[in] pos           地图点的空间位置
     * @param[in] level         特征所在的图层
     * @param[in] pKF           生成这个地图点的关键帧
     * @return ceres::CostFunction*     实例指针
     */
    static ceres::CostFunction* Create(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF) {
        return (new ceres::AutoDiffCostFunction<ReprojErrorOnlyPose, 2, 7>(
                new ReprojErrorOnlyPose(observed_x, observed_y, pos, level,pKF)));
    }

private:
    /// TODO ???
    ceres::CostFunctionToFunctor<2,7> intrinsicReprojErrorOnlyPose_;
};

/**
 * @brief TODO 重投影误差的计算的结构体
 * 
 */
struct ReprojErrorOnlyPoseInvSim3
{
    /**
     * @brief 构造函数
     * 
     * @param[in] observed_x 观测值的x坐标
     * @param[in] observed_y 观测值的y坐标
     * @param[in] pos        地图点的位置
     * @param[in] level      特征所在的图层
     * @param[in] pKF        生成这个地图点的关键帧
     */
    ReprojErrorOnlyPoseInvSim3(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF):
            intrinsicReprojErrorOnlyPoseInvSim3_(new ceres::NumericDiffCostFunction<IntrinsicReprojErrorOnlyPoseInvSim3,ceres::CENTRAL,2,7>(
            new IntrinsicReprojErrorOnlyPoseInvSim3(observed_x, observed_y, pos, level,pKF)))
    {}
    /**
     * @brief 重载括号运算符
     * 
     * @tparam T                TODO 
     * @param[in] camera        相机的内参数 TODO 
     * @param[TODO] residuals     计算得到的残差
     * @return true 
     * @return false 
     */
    template <typename T> bool operator()(const T* const camera, T* residuals) const
    {
        return intrinsicReprojErrorOnlyPoseInvSim3_(camera,residuals);
    }

    /**
     * @brief 创建当前类的一个对象
     * 
     * @param[in] observed_x 观测值的x坐标
     * @param[in] observed_y 观测值的y坐标
     * @param[in] pos        地图点的位置
     * @param[in] level      特征所在的图层
     * @param[in] pKF        生成这个地图点的关键帧
     * @return ceres::CostFunction* 实例指针
     */
    static ceres::CostFunction* Create(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF) {
        return (new ceres::AutoDiffCostFunction<ReprojErrorOnlyPoseInvSim3, 2, 7>(
                new ReprojErrorOnlyPoseInvSim3(observed_x, observed_y, pos, level,pKF)));
    }

private:
    ///TODO  
    ceres::CostFunctionToFunctor<2,7> intrinsicReprojErrorOnlyPoseInvSim3_;
};

/**
 * @brief 和相机内参有关的Sim3误差?  TODO 
 * 
 */
struct IntrinsicRelativeSim3Error
{
    /**
     * @brief 构造函数
     * 
     * @param[in] obs 观测值 TODO 为啥是double的指针型
     */
    IntrinsicRelativeSim3Error(double* obs)
    {
        for (int i = 0; i < 7; ++i)
        {
            mObs[i] = obs[i];
        }
    }


    /**
     * @brief 重载了括号运算符
     * 
     * @param[in] camera1   相机1参数
     * @param[in] camera2   相机2参数  TODO 为什么这里要分相机1和2?
     * @param[out] residual  计算得到的残差
     * @return true 
     * @return false 
     */
    bool operator()(const double* const camera1, const double* const camera2, double* residual) const
    {
        double camera21[7];
        for (int i = 0; i < 7; ++i)
            camera21[i] = mObs[i];

        Eigen::Quaterniond qk1 = Eigen::Quaterniond(camera1[3],camera1[0],camera1[1],camera1[2]);
        Vector3d tk1 = Vector3d(camera1[4],camera1[5],camera1[6]);

        Sophus::Sim3d Sim3_k1(qk1,tk1);

        Eigen::Quaterniond qk2 = Eigen::Quaterniond(camera2[3],camera2[0],camera2[1],camera2[2]);
        Vector3d tk2 = Vector3d(camera2[4],camera2[5],camera2[6]);

        Sophus::Sim3d Sim3_k2(qk2,tk2);

        Eigen::Quaterniond qk21 = Eigen::Quaterniond(camera21[3],camera21[0],camera21[1],camera21[2]);
        Vector3d tk21 = Vector3d(camera21[4],camera21[5],camera21[6]);

        Sophus::Sim3d Sim3_k21(qk21,tk21);

        Sophus::Sim3d result = Sim3_k21*Sim3_k1*(Sim3_k2.inverse());

        //! S21*S1w*Sw2
        double* tResiduals = result.log().data();

        for (int j = 0; j < 7; ++j)
            residual[j] = tResiduals[j];
        return true;
    }

    ///观测值,怀疑是相机参数的观测值?  TODO 可是为什么是7个项呢?
    double mObs[7];

};

/**
 * @brief 相对sim3误差 TODO 
 * 
 */
struct RelativeSim3Error
{
    /**
     * @brief 构造函数
     * 
     * @param[in] obs 观测
     */
    RelativeSim3Error(double* obs):
            intrinsicRelativeSim3Error_(new ceres::NumericDiffCostFunction<IntrinsicRelativeSim3Error,ceres::CENTRAL,7,7,7>(
                    new IntrinsicRelativeSim3Error(obs)))
    {}
    /**
     * @brief 重载括号运算符
     * 
     * @tparam T                TODO
     * @param[in] camera1       相机1的参数
     * @param[in] camera2       相机2的参数
     * @param[out] residuals     计算得到的误差
     * @return true 
     * @return false 
     */
    template <typename T> bool operator()(const T* const camera1, const T* const camera2, T* residuals) const
    {
        return intrinsicRelativeSim3Error_(camera1,camera2,residuals);
    }

    /**
     * @brief 创建当前类的实例
     * 
     * @param[in] obs   观测值
     * @return ceres::CostFunction* 实例指针
     */
    static ceres::CostFunction* Create(double* obs)
    {
        return (new ceres::AutoDiffCostFunction<RelativeSim3Error, 7, 7, 7>(new RelativeSim3Error(obs)));
    }

private:
    /// ???? TODO
    ceres::CostFunctionToFunctor<7,7,7> intrinsicRelativeSim3Error_;

};

}//! namespace ceres

}//! namespace ssvo

#endif