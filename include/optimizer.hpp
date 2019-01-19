#ifndef _OPTIMIZER_HPP_
#define _OPTIMIZER_HPP_

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "map_point.hpp"
#include "keyframe.hpp"
#include "map.hpp"
#include "global.hpp"
//#include "loop_closure.hpp"

namespace ssvo {

//class LoopClosure;

class Optimizer: public noncopyable
{
public:
    typedef std::map<KeyFrame::Ptr,Sophus::Sim3d ,std::less<KeyFrame::Ptr>,
            Eigen::aligned_allocator<std::pair<const KeyFrame::Ptr, Sophus::Sim3d> > > KeyFrameAndPose;

    static void globleBundleAdjustment(const Map::Ptr &map, int max_iters,const uint64_t nLoopKF = 0, bool report=false, bool verbose=false);

    static void motionOnlyBundleAdjustment(const Frame::Ptr &frame, bool use_seeds, bool reject=false, bool report=false, bool verbose=false);

    static void localBundleAdjustment(const KeyFrame::Ptr &keyframe, std::list<MapPoint::Ptr> &bad_mpts, int size=10, int min_shared_fts=50, bool report=false, bool verbose=false);

    static int optimizeSim3(KeyFrame::Ptr pKF1, KeyFrame::Ptr pKF2, std::vector<MapPoint::Ptr> &vpMatches1,
                                  Sophus::Sim3d &S12, const float th2, const bool bFixScale);
    static void OptimizeEssentialGraph(Map::Ptr pMap, KeyFrame::Ptr pLoopKF, KeyFrame::Ptr pCurKF, KeyFrameAndPose &NonCorrectedSim3, KeyFrameAndPose &CorrectedSim3,
                                             const std::map<KeyFrame::Ptr, std::set<KeyFrame::Ptr> > &LoopConnections, const bool &bFixScale = false);

//    static void localBundleAdjustmentWithInvDepth(const KeyFrame::Ptr &keyframe, std::list<MapPoint::Ptr> &bad_mpts, int size=10, bool report=false, bool verbose=false);

    static void refineMapPoint(const MapPoint::Ptr &mpt, int max_iter, bool report=false, bool verbose=false);

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

namespace ceres_slover {
// https://github.com/strasdat/Sophus/blob/v1.0.0/test/ceres/local_parameterization_se3.hpp
class SE3Parameterization : public ceres::LocalParameterization {
public:
    virtual ~SE3Parameterization() {}

    virtual bool Plus(double const *T_raw, double const *delta_raw,
                      double *T_plus_delta_raw) const {
        Eigen::Map<Sophus::SE3d const> const T(T_raw);
        Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
        Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
        T_plus_delta = Sophus::SE3d::exp(delta) * T;
        return true;
    }

    // Set to Identity, for we have computed in ReprojectionErrorSE3::Evaluate
    virtual bool ComputeJacobian(double const *T_raw,
                                 double *jacobian_raw) const {
        Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_raw);
        jacobian.block<6,6>(0, 0).setIdentity();
        jacobian.rightCols<1>().setZero();
        return true;
    }

    virtual int GlobalSize() const { return Sophus::SE3d::num_parameters; }

    virtual int LocalSize() const { return Sophus::SE3d::DoF; }
};

struct ReprojectionError {
    ReprojectionError(double observed_x, double observed_y)
        : observed_x_(observed_x), observed_y_(observed_y) {}

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

    static ceres::CostFunction *Create(const double observed_x, const double observed_y) {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, Sophus::SE3d::num_parameters, 3>(
            new ReprojectionError(observed_x, observed_y)));
    }

    double observed_x_;
    double observed_y_;
};

class ReprojectionErrorSE3 : public ceres::SizedCostFunction<2, 7, 3>
{
public:

    ReprojectionErrorSE3(double observed_x, double observed_y, double weight)
        : observed_x_(observed_x), observed_y_(observed_y), weight_(weight) {}

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

    static inline ceres::CostFunction *Create(const double observed_x, const double observed_y, const double weight = 1.0) {
        return (new ReprojectionErrorSE3(observed_x, observed_y, weight));
    }

private:

    double observed_x_;
    double observed_y_;
    double weight_;

}; // class ReprojectionErrorSE3

class ReprojectionErrorSE3InvDepth : public ceres::SizedCostFunction<2, 7, 7, 1>
{
public:

    ReprojectionErrorSE3InvDepth(double observed_x_ref, double observed_y_ref, double observed_x_cur, double observed_y_cur)
        : observed_x_ref_(observed_x_ref), observed_y_ref_(observed_y_ref),
          observed_x_cur_(observed_x_cur), observed_y_cur_(observed_y_cur) {}

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

    static inline ceres::CostFunction *Create(double observed_x_ref, double observed_y_ref,
                                              double observed_x_cur, double observed_y_cur) {
        return (new ReprojectionErrorSE3InvDepth(observed_x_ref, observed_y_ref, observed_x_cur, observed_y_cur));
    }

private:

    double observed_x_ref_;
    double observed_y_ref_;
    double observed_x_cur_;
    double observed_y_cur_;

};

class ReprojectionErrorSE3InvPoint : public ceres::SizedCostFunction<2, 7, 7, 3>
{
public:

    ReprojectionErrorSE3InvPoint(double observed_x, double observed_y)
        : observed_x_(observed_x), observed_y_(observed_y){}

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

    static inline ceres::CostFunction *Create(double observed_x, double observed_y) {
        return (new ReprojectionErrorSE3InvPoint(observed_x, observed_y));
    }

private:

    double observed_x_;
    double observed_y_;

};

//!**********************************************************************************************************************************************

struct IntrinsicReprojErrorOnlyPose
{
    IntrinsicReprojErrorOnlyPose(double observed_x, double observed_y, double* pos,int level,KeyFrame::Ptr pKF)
    {
        obs_x_ = observed_x;
        obs_y_ = observed_y;
        Mp_Pos_ << pos[0], pos[1], pos[2];
        level_ = level;
        pKF_ = pKF;
    }

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

    double obs_x_, obs_y_;
    Vector3d Mp_Pos_;
    int level_;
    KeyFrame::Ptr pKF_;
};

struct IntrinsicReprojErrorOnlyPoseInvSim3
{
    IntrinsicReprojErrorOnlyPoseInvSim3(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF)
    {
        obs_x_ = observed_x;
        obs_y_ = observed_y;
        Mp_Pos_<< pos[0], pos[1], pos[2];
        level_ = level;
        pKF_ =pKF;
    }

    bool operator()(const double* const camera, double* residuals) const
    {

        Eigen::Quaterniond q = Eigen::Quaterniond(camera[3],camera[0],camera[1],camera[2]);
        Vector3d t = Vector3d(camera[4],camera[5],camera[6]);

        //todo 检查这里的问题
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

    double obs_x_, obs_y_;
    Vector3d Mp_Pos_;
    int level_;
    KeyFrame::Ptr pKF_;
};

struct ReprojErrorOnlyPose
{
    ReprojErrorOnlyPose(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF):
            intrinsicReprojErrorOnlyPose_(new ceres::NumericDiffCostFunction<IntrinsicReprojErrorOnlyPose,ceres::CENTRAL,2,7>(
            new IntrinsicReprojErrorOnlyPose(observed_x,observed_y,pos,level,pKF)))
    {}

    template <typename T> bool operator()(const T* const camera, T* residuals) const
    {
        return intrinsicReprojErrorOnlyPose_(camera,residuals);
    }

    static ceres::CostFunction* Create(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF) {
        return (new ceres::AutoDiffCostFunction<ReprojErrorOnlyPose, 2, 7>(
                new ReprojErrorOnlyPose(observed_x, observed_y, pos, level,pKF)));
    }

private:
    ceres::CostFunctionToFunctor<2,7> intrinsicReprojErrorOnlyPose_;
};

struct ReprojErrorOnlyPoseInvSim3
{
    ReprojErrorOnlyPoseInvSim3(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF):
            intrinsicReprojErrorOnlyPoseInvSim3_(new ceres::NumericDiffCostFunction<IntrinsicReprojErrorOnlyPoseInvSim3,ceres::CENTRAL,2,7>(
            new IntrinsicReprojErrorOnlyPoseInvSim3(observed_x, observed_y, pos, level,pKF)))
    {}

    template <typename T> bool operator()(const T* const camera, T* residuals) const
    {
        return intrinsicReprojErrorOnlyPoseInvSim3_(camera,residuals);
    }

    static ceres::CostFunction* Create(double observed_x, double observed_y, double* pos, int level,KeyFrame::Ptr pKF) {
        return (new ceres::AutoDiffCostFunction<ReprojErrorOnlyPoseInvSim3, 2, 7>(
                new ReprojErrorOnlyPoseInvSim3(observed_x, observed_y, pos, level,pKF)));
    }

private:
    ceres::CostFunctionToFunctor<2,7> intrinsicReprojErrorOnlyPoseInvSim3_;
};

struct IntrinsicRelativeSim3Error
{
    IntrinsicRelativeSim3Error(double* obs)
    {
        for (int i = 0; i < 7; ++i)
        {
            mObs[i] = obs[i];
        }
    }

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

    double mObs[7];

};

struct RelativeSim3Error
{
    RelativeSim3Error(double* obs):
            intrinsicRelativeSim3Error_(new ceres::NumericDiffCostFunction<IntrinsicRelativeSim3Error,ceres::CENTRAL,7,7,7>(
                    new IntrinsicRelativeSim3Error(obs)))
    {}

    template <typename T> bool operator()(const T* const camera1, const T* const camera2, T* residuals) const
    {
        return intrinsicRelativeSim3Error_(camera1,camera2,residuals);
    }

    static ceres::CostFunction* Create(double* obs)
    {
        return (new ceres::AutoDiffCostFunction<RelativeSim3Error, 7, 7, 7>(new RelativeSim3Error(obs)));
    }

private:
    ceres::CostFunctionToFunctor<7,7,7> intrinsicRelativeSim3Error_;

};

}//! namespace ceres

}//! namespace ssvo

#endif