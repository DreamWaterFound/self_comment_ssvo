/**
 * @file local_mapping.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 实现局部建图
 * @version 0.1
 * @date 2019-01-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _SSVO_LOCAL_MAPPING_HPP_
#define _SSVO_LOCAL_MAPPING_HPP_

#include <future>
#include "global.hpp"
#include "feature_detector.hpp"
#include "brief.hpp"
#include "map.hpp"

#ifdef SSVO_DBOW_ENABLE
#include <DBoW3/DBoW3.h>
#include "loop_closure.hpp"
#endif

namespace ssvo{

//需要用到回环检测类的先关内容
class LoopClosure;

/**
 * @brief 局部地图
 * 
 */
class LocalMapper : public noncopyable
{
public:
    ///指向本类的指针
    typedef std::shared_ptr<LocalMapper> Ptr;
    /**
     * @brief 创建初始地图
     * 
     * @param[in] frame_ref 参考帧
     * @param[in] frame_cur 当前帧
     */
    void createInitalMap(const Frame::Ptr &frame_ref, const Frame::Ptr &frame_cur);
    /**
     * @brief 向局部地图中添加关键帧
     * 
     * @param[in] keyframe 要添加的关键帧
     */
    void insertKeyFrame(const KeyFrame::Ptr &keyframe);
    /**
     * @brief 开启(构建局部地图的)主线程
     * 
     */
    void startMainThread();
    /**
     * @brief 关闭(构建局部地图的)主线程
     * 
     */
    void stopMainThread();
    /**
     * @brief 增加需要优化的地图点
     * 
     * @param[in] mpt 地图点
     */
    void addOptimalizeMapPoint(const MapPoint::Ptr &mpt);
    /**
     * @brief 精确话地图点,其实就是对地图点进行优化
     * 
     * @param[in] max_optimalize_num    最大优化次数
     * @param[in] outlier_thr           外点阈值
     * @return int                      优化后的点个数 ?  TODO 
     */
    int refineMapPoints(const int max_optimalize_num = -1, const double outlier_thr = 2.0/480.0);
    /**
     * @brief 从深度滤波器的种子中创建特征
     * 
     * @param[in] seed 给定的种子
     */
    void createFeatureFromSeed(const Seed::Ptr &seed);
    /**
     * @brief 进行重定位
     * @details 这就是我们仨需要做的!! TODO FIXME: 补充这里 
     * @param[in] frame         给定的当前帧
     * @param[in] corners       当前帧中的角点
     * @return KeyFrame::Ptr    匹配上的最近的关键帧
     */
    KeyFrame::Ptr relocalizeByDBoW(const Frame::Ptr &frame, const Corners &corners);
    /**
     * @brief 创建局部地图
     * @detials 这种方式不会使用闭环
     * @param[in] fast              fast角点提取器
     * @param[in] report            是否汇报
     * @param[in] verbose           是否产生详情
     * @return LocalMapper::Ptr     实例指针
     */
    static LocalMapper::Ptr create(const FastDetector::Ptr fast, bool report = false, bool verbose = false)
    { return LocalMapper::Ptr(new LocalMapper(fast, report, verbose));}

#ifdef SSVO_DBOW_ENABLE
    /**
     * @brief 创建局部地图
     * 
     * @param[in] vocabulary        词袋
     * @param[in] database          词袋数据库 TODO 和上面的有什么区别?
     * @param[in] fast              fast角点提取器
     * @param[in] report            是否汇报
     * @param[in] verbose           是否产生详情
     * @return LocalMapper::Ptr     实例指针
     */
    static LocalMapper::Ptr create(DBoW3::Vocabulary* vocabulary, DBoW3::Database* database, const FastDetector::Ptr fast, bool report = false, bool verbose = false)
    { return LocalMapper::Ptr(new LocalMapper(vocabulary, database, fast, report, verbose));}
    /**
     * @brief 设置回环检测器
     * 
     * @param[in] loop_closure 回环检测器句柄
     */
    void setLoopCloser(std::shared_ptr<LoopClosure> loop_closure);
#endif
    /**
     * @brief 停止局部建图的工作,其实就是停止局部建图的线程
     * 
     */
    void setStop();
    /**
     * @brief 查看是否有停止局部建图线程的请求
     * 
     * @return true 
     * @return false 
     */
    bool isRequiredStop();
    /**
     * @brief 释放啥啊 TODO 
     * 
     */
    void release();
    /**
     * @brief TODO 
     * 
     * @return true 
     * @return false 
     */
    bool finish_once();
private:
    /**
     * @brief 构造函数
     * @detials 这种方式不会使用闭环
     * @param[in] fast      fast角点提取器
     * @param[in] report    是否汇报
     * @param[in] verbose   是否产生详情
     */
    LocalMapper(const FastDetector::Ptr fast, bool report, bool verbose);

#ifdef SSVO_DBOW_ENABLE
    /**
     * @brief 构造函数
     * @details 这种方式是需要使用到闭环的
     * @param[in] vocabulary    词袋
     * @param[in] database      词袋数据库 
     * @param[in] fast          fast角点提取器
     * @param[in] report        是否汇报
     * @param[in] verbose       是否产生详情
     */
    LocalMapper(DBoW3::Vocabulary* vocabulary, DBoW3::Database* database,const FastDetector::Ptr fast, bool report, bool verbose);
    ///指向闭环检测器的指针
    std::shared_ptr<LoopClosure> loop_closure_;
#endif
    /**
     * @brief 局部建图线程的主函数 TODO 
     * 
     */
    void run();
    /**
     * @brief 检查新的关键帧
     * @detials TODO 检查什么内容?
     * @return KeyFrame::Ptr    返回的是什么关键帧的指针? TODO
     */
    KeyFrame::Ptr checkNewKeyFrame();
    /**
     * @brief TODO 什么叫做完成最近的关键帧? 完成对最近的关键帧的数据的处理?
     * 
     */
    void finishLastKeyFrame();  
    /**
     * @brief 从种子所携带的特征来...创建自己的特征??? TODO 
     * 
     * @param[in] keyframe 关键帧
     * @return int          TODO ???
     */
    int createFeatureFromSeedFeature(const KeyFrame::Ptr &keyframe);
    /**
     * @brief 从局部地图创建特征??? TODO
     * 
     * @param[in] keyframe  关键帧
     * @param[in] num       TODO 创建的特征个数?
     * @return int          TODO 实际创建的特征个数?
     */
    int createFeatureFromLocalMap(const KeyFrame::Ptr &keyframe, const int num = 5);
    /**
     * @brief TODO 剔除???
     * 
     * @param[in] keyframe  关键帧 
     */
    void checkCulling(const KeyFrame::Ptr &keyframe);
    /**
     * @brief 将什么添加到数据库?
     * 
     * @param[in] keyframe 关键帧
     */
    void addToDatabase(const KeyFrame::Ptr &keyframe);

public:

    ///局部地图对象
    Map::Ptr map_;

private:

    ///局部地图管理器的各种选项
    struct Option{
        double min_disparity;           ///<最小视差
        int min_redundant_observations; ///<最小的重复观测数目
        int max_features;               ///<最大的特征数
        int num_reproject_kfs;          ///<重投影的帧数 TODO ?
        int num_local_ba_kfs;           ///<进行局部BA的关键帧个数
        int min_local_ba_connected_fts; ///<最小的局部BA所连接的特征点个数?
        int num_align_iter;             ///<什么对齐的迭代次数? TODO 
        double max_align_epsilon;       ///<最大的对齐误差
        double max_align_error2;        ///<最大对齐误差的平方
        double min_found_ratio_;        ///<找回率
    } options_;

    ///fast角点提取器
    FastDetector::Ptr fast_detector_;
    ///brief描述子计算器
    BRIEF::Ptr brief_;
    ///一个关键帧缓冲队列 用来干嘛的?  TODO
    std::deque<KeyFrame::Ptr> keyframes_buffer_;
    ///最近的关键帧
    KeyFrame::Ptr keyframe_last_;

#ifdef SSVO_DBOW_ENABLE
    ///闭环检测中使用, 词袋
    DBoW3::Vocabulary* vocabulary_;
    ///闭环检测中使用,词袋的数据库
    DBoW3::Database* database_;

#endif
    ///是否汇报
    const bool report_;
    ///是否产生详情
    const bool verbose_;

    ///局部构图的线程
    std::shared_ptr<std::thread> mapping_thread_;
    ///一个列表,存储有需要进行优化的地图点
    std::list<MapPoint::Ptr> optimalize_candidate_mpts_;

    ///停止当前构图线程的请求标志
    bool stop_require_;
    ///TODO ????
    bool finish_once_;

    ///TODO 不知道用来做什么的线程锁
    std::mutex mutex_stop_;
    std::mutex mutex_keyframe_;
    std::mutex mutex_optimalize_mpts_;
    ///用于线程间通讯的条件变量
    std::condition_variable cond_process_;

};

}

#endif //_SSVO_LOCAL_MAPPING_HPP_
