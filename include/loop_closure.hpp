//
// Created by jh on 18-11-29.
//

/**
 * @file loop_closure.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 姜浩师兄增加的闭环处理的部分
 * @version 0.1
 * @date 2019-01-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef SSVO_LOOP_CLOSURE_HPP
#define SSVO_LOOP_CLOSURE_HPP

#include "global.hpp"
#include "DBoW3/DBoW3.h"
#include "DBoW3/DescManip.h"
#include "keyframe.hpp"
#include "sim3_solver.hpp"
#include "brief.hpp"
#include <eigen3/Eigen/Dense>
#include "local_mapping.hpp"

namespace ssvo{

class LocalMapper;

/**
 * @brief 闭环检测类
 * @todo 其实我觉得,这个类应该继承 ssvo::noncopyable 类
 */
class LoopClosure
{

public:
    ///一致性...组?? TODO 
    typedef std::pair<std::set<KeyFrame::Ptr>,int> ConsistentGroup;
    ///map类型,通过关键帧可以索引到相关联的位姿
    typedef std::map<KeyFrame::Ptr,
                     Sophus::Sim3d ,
                     std::less<KeyFrame::Ptr>,
                     Eigen::aligned_allocator<
                                    std::pair<const KeyFrame::Ptr, Sophus::Sim3d> 
                                            > 
                    > KeyFrameAndPose;
    ///管理本类的指针类型
    typedef std::shared_ptr<LoopClosure> Ptr;
    /**
     * @brief 开始运行本类的主运行线程
     * 
     */
    void startMainThread();
    /**
     * @brief 停止本类的主线程
     * 
     */
    void stopMainThread();
    /**
     * @brief 插入关键帧 TODO 在回环中插入关键帧,是什么意思?
     * 
     * @param[in] kf 要插入的关键帧
     */
    void insertKeyFrame(KeyFrame::Ptr kf);
    /**
     * @brief 回环检测的主线程
     * 
     */
    void run();

    /**
     * @brief 创建当前的类对象
     * 
     * @param[in] vocabulary    词袋字典
     * @param[in] database      词袋数据库
     * @return Ptr              实例指针
     */
    inline static Ptr creat(DBoW3::Vocabulary* vocabulary, DBoW3::Database* database)
    { return Ptr(new LoopClosure(vocabulary, database));}

    /**
     * @brief 设置局部地图
     * 
     * @param[in] local_mapper  局部地图句柄 
     */
    void setLocalMapper(std::shared_ptr<LocalMapper> local_mapper);
    /**
     * @brief 判断当前是否在进行全局BA的操作
     * 
     * @return true 
     * @return false 
     */
    bool isRunningGBA(){
        std::unique_lock<std::mutex> lock(mutex_GBA_);
        return RunningGBA_;
    }
    /**
     * @brief 判断当前全局BA的操作是否已经结束了
     * 
     * @return true 
     * @return false 
     */
    bool isFinishedGBA(){
        std::unique_lock<std::mutex> lock(mutex_GBA_);
        return FinishedGBA_;
    }

public:
    ///保存有关键帧的列表
    std::list<KeyFrame::Ptr> keyFramesList_; //list-快速的插入和删除，可以在两端进行操作 [师兄添加]
    ///保存有关键帧的向量    TODO 为什么这里会有这两种变量
    std::vector<KeyFrame::Ptr> KeyFrames;
    /// TODO
    std::mutex mutex_database_;
    /// TODO 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// 最近的构成闭环的关键帧的id
    uint64_t LastLoopKFid_;
    ///全局BA完成的结束标志 TODO 我猜测是这个样子
    bool update_finish_;


private:
    /**
     * @brief 构造函数
     * 
     * @param[in] vocabulary    词袋字典
     * @param[in] database_     词袋数据库
     */
    LoopClosure(DBoW3::Vocabulary* vocabulary, DBoW3::Database* database_);
    /**
     * @brief 检查新的关键帧?  TODO
     * 
     * @return true 
     * @return false 
     */
    bool CheckNewKeyFrames();
    /**
     * @brief 检查回环检测
     * 
     * @return true 
     * @return false 
     */
    bool DetectLoop();
    /**
     * @brief 计算sim3  TODO 那么之前计算的是什么鬼?
     * 
     * @return true 
     * @return false 
     */
    bool ComputeSim3();
    /**
     * @brief 纠正回环? TODO 
     * 
     */
    void CorrectLoop();
    /**
     * @brief 通过词袋来搜索关键帧
     * 
     * @param[in] loopKeyFrame     要进行搜索和匹配的关键帧
     * @param[in] Matches12         ???? TODO
     * @param[TODO] bestidx        搜索到的关键帧的id
     * @return int                  TODO 
     */
    int SearchByBoW(KeyFrame::Ptr loopKeyFrame, std::vector<MapPoint::Ptr> &Matches12, std::vector<int > &bestidx);
    /**
     * @brief 通过给定的sim3变换来搜索
     * 
     * @param[in] loopKeyFrame      要搜索和匹配的关键帧
     * @param[in] matches_1_2       TODO
     * @param[in] matches_2_1       TODO
     * @param[in] s12               缩放洗漱
     * @param[in] R12               旋转向量
     * @param[in] t12               平移向量
     * @param[in] th                TODO 
     * @param[in] Matches12         TODO 连接到的地图点?
     * @param[out] bestidx          搜索得到的最佳的关键帧的id
     * @return int                  TODO 猜测可能和上面的相同
     */
    int SearchBySim3(KeyFrame::Ptr loopKeyFrame,
                     std::unordered_map<uint64_t,uint64_t > &matches_1_2,std::unordered_map<uint64_t,uint64_t > &matches_2_1,
                     double &s12, const Matrix3d &R12, const Vector3d &t12, float th, std::vector<MapPoint::Ptr> &Matches12,  std::vector<int > &bestidx);
    /**
     * @brief 通过投影搜索
     *  
     * @param[in] th    ???? TODO 
     * @return int      TODO 
     */
    int SearchByProjection(int th );
    /**
     * @brief 在给定的关键帧的集合中搜索并且进行融合
     * 
     * @param[in] CorrectedPosesMap 关键帧集合
     */
    void searchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);
    /**
     * @brief 闭环^融合? TODO 
     * 
     * @param[in] pKF               关键帧
     * @param[in] Scw               世界坐标系下的当前关键帧的Sim3位姿表示
     * @param[in] th                TODO
     * @param[TODO] vpReplacePoint  被取代的地图点?? TODO 
     * @return int                  TODO 
     */
    int fuse(KeyFrame::Ptr pKF,const Sophus::Sim3d& Scw, float th, std::vector<MapPoint::Ptr > &vpReplacePoint);
    /**
     * @brief 调用全局BA的线程
     * 
     * @param[in] nLoopKF TODO ????
     */
    void RunGlobalBundleAdjustment(uint64_t nLoopKF);

private:
    /**
     * @brief 检测可能出现闭环关系的候选关键帧
     * 
     * @param[in] minScore                      给定的最小评分
     * @return std::vector<KeyFrame::Ptr>       得到的候选关键帧的集合
     */
    std::vector<KeyFrame::Ptr> DetectLoopCandidates(double minScore);

    ///词袋模型的字典句柄
    DBoW3::Vocabulary* vocabulary_;
    ///词典模型的数据库句柄
    DBoW3::Database* database_;

    ///当前的关键帧句柄
    KeyFrame::Ptr curKeyFrame_;
    ///匹配到的关键帧的句柄
    KeyFrame::Ptr MatchedKeyFrame_;
    ///一致图的集合  TODO 但是我还是不明白这个是做什么的
    std::vector<ConsistentGroup> mvConsistentGroups;
    ///TODO 
    std::vector<KeyFrame::Ptr> mvpEnoughConsistentCandidates;
    ///和当前关键帧具有连接关系的关键帧的集合
    std::vector<KeyFrame::Ptr> mvpCurrentConnectedKFs;
    ///和当前帧匹配的地图点
    std::vector<MapPoint::Ptr> CurrentMatchedPoints; //当前匹配的mappoint [师兄注释,下面也是]
//    *   [mpt1_1  mpt_a_2 ]  一一匹配
//    *   [mpt2_1  mpt_b_2 ]
//    *   [mpt3_1  mpt_c_2 ]
//    *   [mpt4_1  mpt_d_2 ]
//    *   [mpt5_1  mpt_e_m ]
//    *   [mpt6_1  mpt_f_n ]
//    *   [mpt7_1  mpt_g_n ]
//    *   [mpt8_1  mpt_h_k ]
    ///闭环帧和它的共视帧观测到的地图点
    std::vector<MapPoint::Ptr> LoopMapPoints; //闭环帧及其共视帧的所有观测到的mpt，设置为set，不包含相同的元素[师兄注释]


    ///TODO 不知道是谁的sim3位姿
    Sophus::Sim3d sim3_cw;
    ///通过闭环关系求得的当前帧的位姿
    SE3d T_cw; //通过闭环帧求得的当前帧的位姿，而不是自身的Tcw[师兄注释]
    ///回环检测线程指针
    std::shared_ptr<std::thread> loop_closure_thread_;
    ///线程锁
    std::mutex mutex_keyFramesList_;

    ///连续性检测的阈值
    double mnCovisibilityConsistencyTh; //连续性检测的阈值[师兄注释]
    ///当前回环检测的全局BA过程是否完成?  TODO 
    bool ifFinished;
    ///局部地图句柄
    std::shared_ptr<LocalMapper> local_mapper_;


    // Variables related to Global Bundle Adjustment
    ///是否正在运行全局BA
    bool RunningGBA_;
    ///全局BA的操作是否已经结束 TODO 奇怪,使用上面的那个变量就足够了判断了,为什么还要加上这个变量?
    bool FinishedGBA_;
    //TODO  还要在迭代的时候设置是否停止的标志位[师兄注释]
    ///是否有请求停止全局BA的标志,还是当前全局BA是否停止的标志?
    bool StopGBA_;
    ///线程锁,全局BA过程使用
    std::mutex mutex_GBA_;
    ///全局BA的线程
    std::thread* thread_GBA_;
    ///TODO 
    bool FullBAIdx_;

    //! 通过最小得分计算的闭环次数，仅用于输出信息[师兄注释]
    ///通过最小得分计算的闭环次数，仅用于输出信息
    int loop_time_;
    ///用于线程间通讯的条件变量
    std::condition_variable cond_process_;
};

}



#endif //SSVO_LOOP_CLOSURE_HPP
