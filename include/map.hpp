/**
 * @file map.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 定义了地图这个数据类型
 * @version 0.1
 * @date 2019-01-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _MAP_HPP_
#define _MAP_HPP_

#include "map_point.hpp"
#include "keyframe.hpp"
#include "global.hpp"

namespace ssvo{

class LocalMapper;
class DepthFilter;

/**
 * @brief 地图
 * @detials 局部建图器和深度滤波器是它的友元类
 */
class Map: public noncopyable
{
    //NOTICE 友元类
    friend class LocalMapper;
    friend class DepthFilter;

public:
    ///指向当前类的指针类型
    typedef std::shared_ptr<Map> Ptr;

    /**
     * @brief 获取创建地图的指定id的关键帧
     * 
     * @param[in] id            指定的id
     * @return KeyFrame::Ptr    关键帧句柄
     */
    KeyFrame::Ptr getKeyFrame(uint64_t id);
    /**
     * @brief 获取所有的关键帧
     * 
     * @return std::vector<KeyFrame::Ptr>  存储有所有关键帧的列表
     */
    std::vector<KeyFrame::Ptr> getAllKeyFrames();
    /**
     * @brief 获取地图中的所有地图点
     * 
     * @return std::vector<MapPoint::Ptr> 地图中的所有地图点
     */
    std::vector<MapPoint::Ptr> getAllMapPoints();

    /**
     * @brief 获取地图中关键帧的个数
     * 
     * @return uint64_t 地图中关键帧的个数
     */
    uint64_t KeyFramesInMap();
    /**
     * @brief 获取地图中地图点的个数
     * 
     * @return uint64_t 地图中地图点的个数
     */
    uint64_t MapPointsInMap();

private:

    /**
     * @brief 清空地图
     * 
     */
    void clear();
    /**
     * @brief 向地图中添加关键帧
     * @detials 这意味着什么? 会生成地图点吗??? TODO 
     * @param[in] kf 要添加的关键帧
     * @return true 
     * @return false 
     */
    bool insertKeyFrame(const KeyFrame::Ptr &kf);
    /**
     * @brief 从地图中删除关键帧
     * @details 也会顺带把他们所生成的地图点从地图中移除吗 TODO 
     * @param[in] kf 要删除的关键帧
     */
    void removeKeyFrame(const KeyFrame::Ptr &kf);
    /**
     * @brief 向地图中插入地图点
     * 
     * @param[in] mpt 要插入的地图点
     */
    void insertMapPoint(const MapPoint::Ptr &mpt);
    /**
     * @brief 在地图中删除地图点
     * 
     * @param[in] mpt   要删除的地图点
     */
    void removeMapPoint(const MapPoint::Ptr &mpt);
    /**
     * @brief 构造函数
     * 
     * @return Map::Ptr 实例指针
     */
    inline static Map::Ptr create() {return Map::Ptr(new Map());}

public:
    /**
     * @brief 存储了已经删除的地图点
     * @detials 为什么我们并没有真正地删除掉他们? TODO ORB中貌似也是类似的操作
     */
    std::set<MapPoint::Ptr> removed_mpts_;

    ///一系列有关的线程锁
    std::mutex mutex_kf_;
    std::mutex mutex_mpt_;
    std::mutex mutex_update_;

private:

    ///存储了这个地图中有关的关键帧
    std::unordered_map<uint64_t, KeyFrame::Ptr> kfs_;
    ///存储了这个地图中的地图点
    std::unordered_map<uint64_t, MapPoint::Ptr> mpts_;


}; //class Map

}   //namespace ssvo

#endif