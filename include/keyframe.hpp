/**
 * @file keyframe.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 关键帧的定义文件
 * @version 0.1
 * @date 2019-01-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _KEYFRAME_HPP_
#define _KEYFRAME_HPP_

#include "global.hpp"
#include "frame.hpp"
#include "DBoW3/DBoW3.h"

namespace ssvo
{

class Map;

/**
 * @brief 关键帧
 * @detials 第二个继承的是什么啊,C++11特性,好像是为了能够实现智能指针共享的东西 TODO 
 * @see https://blog.csdn.net/caoshangpa/article/details/79392878
 */
class KeyFrame: public Frame, public std::enable_shared_from_this<KeyFrame>
{
public:
    ///指向当前类的指针
    typedef std::shared_ptr<KeyFrame> Ptr;
    /**
     * @brief 更新关键帧之间的链接关系? TODO 
     * 
     */
    void updateConnections();
    /**
     * @brief 设置当前关键帧是bad
     * 
     */
    void setBad();
    /**
     * @brief 判断当前帧是否是bad
     * 
     * @return true 
     * @return false 
     */
    bool isBad();
    /**
     * @brief 获取和本关键帧具有连接关系的关键帧
     * 
     * @param[TODO] num      具有连接关系的关键帧数目? TODO 
     * @param[TODO] min_fts  TODO 
     * @return std::set<KeyFrame::Ptr> 具有连接关系的关键帧的组合 
     */
    std::set<KeyFrame::Ptr> getConnectedKeyFrames(int num=-1, int min_fts = 0);
    /**
     * @brief 获取具有二级链接关系的关键帧? TODO 
     * 
     * @param[TODO] num                 ??? 
     * @return std::set<KeyFrame::Ptr>  符合条件的关键帧的集合
     */
    std::set<KeyFrame::Ptr> getSubConnectedKeyFrames(int num=-1);
    /**
     * @brief 获取获取具有方向的二级链接关系的关键帧? TODO
     * 
     * @return std::set<KeyFrame::Ptr> 符合条件的关键帧集合
     */
    std::set<KeyFrame::Ptr> getOrderedSubConnectedKeyFrames();

    //! roughly, only can be used in loopclosure
    /**
     * @brief 获取指定区域中的特征点数目
     * @detials 师兄说目前这个函数只是粗略的写的,并且目前仅仅在回环检测的时候才会被用到
     * @param[in] x                 区域中心x坐标
     * @param[in] y                 区域中心y坐标
     * @param[in] r                 区域的半径
     * @return std::vector<int >    在这个区域中的特征点的id
     */
    std::vector<int > getFeaturesInArea(const double &x, const double &y, const double &r);
    /**
     * @brief 光流图像金字塔
     * @detials TODO 什么是光流图像金字塔?
     * @notes 这个函数目前暂时被禁用了
     * @return const ImgPyr 得到的光流图像金字塔
     */
    const ImgPyr opticalImages() const = delete;    //! disable this function
    /**
     * @brief 构造函数
     * @detials 从给定的普通帧来创建关键帧,也可以说设置某个普通帧为关键帧
     * @param[in] frame         帧
     * @return KeyFrame::Ptr    实例指针
     */
    inline static KeyFrame::Ptr create(const Frame::Ptr frame)
    { return Ptr(new KeyFrame(frame)); }
    /**
     * @brief 设置不可被删除? TODO 
     * 
     */
    void setNotErase();
    /**
     * @brief 设置本关键帧可以被删除? TODO 
     * 
     */
    void setErase();
    /**
     * @brief 增加回环检测的边
     * @notes 估计是后来加的
     * @param[in] pKF 和本关键帧相关联的关键帧  
     */
    void addLoopEdge(KeyFrame::Ptr pKF);
    /**
     * @brief 获取和某个关键帧连接的权重
     *  
     * @param[in] pKF   某个和本帧关联的关键帧 
     * @return int      权重大小  TODO 为什么不是浮点型数据啊?
     */
    int getWight(KeyFrame::Ptr pKF);
    /**
     * @brief 获取当前关键帧的父关键帧 TODO
     * 
     * @return KeyFrame::Ptr 当前关键帧的父关键帧
     */
    KeyFrame::Ptr getParent();
    /**
     * @brief 获取回环边上的关键帧
     * 
     * @return std::set<KeyFrame::Ptr> 回环边上关键帧的序列 
     */
    std::set<KeyFrame::Ptr> getLoopEdges();

private:
    /**
     * @brief 构造函数
     * 
     * @param[in] frame 作为关键帧的普通帧
     */
    KeyFrame(const Frame::Ptr frame);
    /**
     * @brief 给本关键帧添加连接关系
     * 
     * @param[in] kf        和哪个关键帧添加连接关系
     * @param[in] weight    这个连接关系的权重
     */
    void addConnection(const KeyFrame::Ptr &kf, const int weight);
    /**
     * @brief 更新有向连接(权重)??? TODO 
     * 
     */
    void updateOrderedConnections();
    /**
     * @brief 移除某个关键帧和本关键帧的连接关系
     * 
     * @param[in] kf 指定的关键帧
     */
    void removeConnection(const KeyFrame::Ptr &kf);

public:

    ///下一个关键帧的id
    static uint64_t next_id_;
    ///当前关键帧的id 
    const uint64_t frame_id_;
    /// 词袋中的特征??? TODO
    std::vector<Feature::Ptr> dbow_fts_;
    ///当前帧中的特征点的描述子 TODO 
    cv::Mat descriptors_;
    ///词袋的id>???  TODO 
    unsigned int dbow_Id_;
    ///词袋向量? TODO 
    DBoW3::BowVector bow_vec_;
    ///词袋的特征向量
    DBoW3::FeatureVector feat_vec_;


    //TODO 可能有特征点融合的问题 [师兄加的]
    ///地图点的id和对应的描述子
    std::unordered_map<uint64_t , cv::Mat> mptId_des;
    ///地图点的id和对应的节点id??? TODO 
    std::unordered_map<uint64_t , uint> mptId_nodeId;
    ///地图点的id和在世界坐标系下的id??? TODO 
    std::unordered_map<uint64_t , uint > mptId_wordId;
    ///回环队列? 为什么是这种数据格式??? TODO
    uint64_t loop_query_;
    ///TODO 感觉和优化有关?
    uint64_t GBA_KF_;

    ///存储本帧中的特征点
    std::vector<cv::KeyPoint> KeyPoints;

    //解决mpt和feature无序的问题 [师兄添加]
    ///在词袋中的特征 TODO 
    std::vector<Feature::Ptr> featuresInBow;
    ///在词袋中的地图点 TODO 
    std::vector<MapPoint::Ptr> mapPointsInBow;
private:

    ///和当前关键帧有连接关系的关键帧
    std::map<KeyFrame::Ptr, int> connectedKeyFrames_;

    //todo remove from database [师兄加的] TODO 
    ///词袋数据库 TODO ????
    DBoW3::Database* mpDatabase_;

    ///存储有向的连接的关键帧
    std::multimap<int, KeyFrame::Ptr> orderedConnectedKeyFrames_;
    ///标记本帧是否是bad的
    bool isBad_;
    ///线程锁, 好像是和关键帧的连接关系有关的
    std::mutex mutex_connection_;
    ///标记本帧不要删除
    bool notErase;
    ///哈???? TODO 
    bool toBeErase;
    ///回环边?? TODO 
    std::set<KeyFrame::Ptr> loopEdges_;

    //TODO 删除（bad）的时候记着改 [TODO]
    ///当前关键帧的负关键帧
    KeyFrame::Ptr parent_;

};// class KeyFrame

} //namespace ssvo

#endif