/**
 * @file map_point.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 地图点的实现
 * @version 0.1
 * @date 2019-01-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef _MAP_POINT_HPP_
#define _MAP_POINT_HPP_

#include "feature.hpp"
#include "global.hpp"

namespace ssvo {

class Frame;

class KeyFrame;

/**
 * @brief 地图点
 * 
 */
class MapPoint : public std::enable_shared_from_this<MapPoint>
{
public:

    /**
     * @brief 类型定义
     * 
     */
    enum Type{
        SEED = 0,   ///<种子
        STABLE = 1, ///<稳定的
        BAD = 2,    ///<发散的
    };
    ///指向当前类对象的指针类型
    typedef std::shared_ptr<MapPoint> Ptr;
    ///指向关键帧的指针类型
    typedef std::shared_ptr<KeyFrame> KeyFramePtr;
    ///指向帧的指针类型
    typedef std::shared_ptr<Frame> FramePtr;

    /**
     * @brief 获得当前的这个地图点的状态类型
     * @see ssvo::MapPoint::Type
     * @return Type 当前这个状态点的状态类型
     */
    Type type();
    /**
     * @brief 设置这个地图点的状态为发散
     * 
     */
    void setBad();
    /**
     * @brief 判断这个地图点的状态是否是发散
     * 
     * @return true 
     * @return false 
     */
    bool isBad();
    /**
     * @brief 复位这个地图点的状态
     * 
     * @param[in] type 要设置成的这个地图点的状态?  TODO?
     */
    void resetType(Type type);
    /**
     * @brief 获取生成这个地图点的参考关键帧
     * @todo 是不是说,地图点都是从关键帧中生成的?
     * @return KeyFramePtr 
     */
    KeyFramePtr getReferenceKeyFrame();
    /**
     * @brief 融合???? TODO
     * 
     * @param[in] mpt   地图点
     * @param[in] loop  TODO ????
     * @return true     融合成功
     * @return false    融合失败
     */
    bool fusion(const MapPoint::Ptr &mpt,const bool loop = false);
    /**
     * @brief 地图点的替代 ??? TODO 
     * 
     * @param[in] mpt 地图点
     * @return true     替代成功
     * @return false    替代失败
     */
    bool replace(const MapPoint::Ptr &mpt);
    /**
     * @brief 增加档前地图点的观测
     * 
     * @param[in] kf 关键帧
     * @param[in] ft 关键帧的特征? TODO 为什么需要这个东西
     */
    void addObservation(const KeyFramePtr &kf, const Feature::Ptr &ft);
    /**
     * @brief 啥观测啊,是对档前的这个地图点进行观测吗?? TODO 
     * 
     * @return int TODO ????
     */
    int observations();
    /**
     * @brief 获取当前地图点的所有观测
     * @detials 这里所说的观测貌似是只对关键帧有效 TODO 
     * @return std::map<KeyFramePtr, Feature::Ptr>  观测到这个地图点的关键帧序列
     */
    std::map<KeyFramePtr, Feature::Ptr> getObservations();
    /**
     * @brief 从当前地图点上移除一个观测
     * 
     * @param[in] kf    要移除的观测的关键帧
     * @return true 
     * @return false 
     */
    bool removeObservation(const KeyFramePtr &kf);
    /**
     * @brief 寻找观测
     * 
     * @param[in] kf 关键帧
     * @return Feature::Ptr 从这个关键帧中观测得到的特征??? TODO ???
     */
    Feature::Ptr findObservation(const KeyFramePtr kf);
    /**
     * @brief 更新视角和尺度????  谁的视角,谁的尺度?? TODO 
     * 
     */
    void updateViewAndDepth();
    /**
     * @brief 获取这个地图点的描述子
     * @details 所以说,地图点有它自己的描述子咯?
     * @return std::vector<cv::Mat > 这个地图点的描述子
     */
    std::vector<cv::Mat > getDescriptors();
    /**
     * @brief 预测尺度??? 初始化的时候不就已经确定了尺度了吗,为什么在这里还需要进行预测?? TODO 
     * 
     * @param[in] dist          距离?? TODO 
     * @param[in] max_level     最大图层??? TODO 
     * @return int              TODO ?????
     */
    int predictScale(const double dist, const int max_level);
    /**
     * @brief 预测尺度?? TODO
     * 
     * @param[in] dist_ref  TODO 什么距离?
     * @param[in] dist_cur  TODO 
     * @param[in] level_ref TODO 参考帧的图层???
     * @param[in] max_level TODO 最大层???
     * @return int          TODO ????
     */
    static int predictScale(const double dist_ref, const double dist_cur, const int level_ref, const int max_level);
    /**
     * @brief 获取最小的尺度不变性???  TODO 为什么尺度不变性还有多个????
     * 
     * @return double 所谓的最小的尺度不变性 TODO 
     */
    double getMinDistanceInvariance();

    /**
     * @brief 获取最大的尺度不变性 TODO 
     * 
     * @return double 最大的尺度不变性
     */
    double getMaxDistanceInvariance();
    /**
     * @brief 获取最近的观测??? TODO 
     * 
     * @param[in] frame         TODO ?
     * @param[TODO] keyframe    TODO ???
     * @param[in] level         TODO ???? 得到观测的图层?
     * @return true     
     * @return false 
     */
    bool getCloseViewObs(const FramePtr &frame, KeyFramePtr &keyframe, int &level);
    /**
     * @brief ???? TODO 
     * 
     * @param[in] n TODO 
     */
    void increaseFound(int n=1);
    /**
     * @brief 什么叫做增加可见性? TODO ? 它和上面的found又有什么不同?
     * 
     * @param[in] n TODO ?
     */
    void increaseVisible(int n=1);
    /**
     * @brief 获取Found
     * 
     * @return uint64_t Found 
     */
    uint64_t getFound();
    /**
     * @brief 获取可视的次数
     * 
     * @return uint64_t 可视的次数
     */
    uint64_t getVisible();
    /**
     * @brief 获取被找到的比例
     * 
     * @return double 被找到的比例
     */
    double getFoundRatio();
    /**
     * @brief 获取观测的???? TODO 
     * @return Vector3d TODO ????
     */
    Vector3d getObsVec();
    /**
     * @brief 设置当前地图点的位置
     * 
     * @param[in] x 坐标x
     * @param[in] y 坐标y
     * @param[in] z 坐标z
     */
    inline void setPose(const double x, const double y, const double z)
    {
        pose_[0] = x;
        pose_[1] = y;
        pose_[2] = z;
    }

    /**
     * @brief 设置当前地图点的位置
     * 
     * @param[in] pose 以向量形式表示的位置
     */
    inline void setPose(const Vector3d pose) { pose_ = pose; }
    /**
     * @brief 获取当前地图点的位置
     * 
     * @return Vector3d 
     */
    inline Vector3d pose() { return pose_; }

    /**
     * @brief 构造函数
     * 
     * @param[in] p 地图点的位置
     * @return Ptr  实例指针
     */
    inline static Ptr create(const Vector3d &p)
    { return Ptr(new MapPoint(p)); }

private:
    /**
     * @brief 构造函数
     * 
     * @param[in] p 地图点的位置
     */
    MapPoint(const Vector3d &p);

    /**
     * @brief 从参考关键帧中更新
     * @details 更新什么呢?
     */
    void updateRefKF();

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ///下一个地图点的id
    static uint64_t next_id_;
    ///当前地图点的id
    const uint64_t id_;
    //是不是因为要将地图点放在什么容器中,这个容器所要求的id啊

    ///TODO ???
    uint64_t loop_id_;
    ///TODO ???
    uint64_t GBA_KF_;
    ///TODO 这个应该是后来闭环添加的吧, 被多少关键帧所矫正
    uint64_t mnCorrectedByKF;
    ///TODO 被多少参考帧所矫正?
    uint64_t mnCorrectedReference;
    ///TODO  ???
    static const double log_level_factor_;

    ///优化后的地图点的位置
    Vector3d optimal_pose_;
    ///TODO ?
    double optimal_inv_z_;
    ///TODO ????
    uint64_t last_structure_optimal_;

private:
    ///这个地图点的位置,虽然只是表示位置,但是变量名依旧起成了pose_
    Vector3d pose_;
    ///在不同的关键帧中得到的对当前帧的观测(观测结果是以特征描述子的形式保存的)
    std::unordered_map<KeyFramePtr, Feature::Ptr> obs_;

    ///当前特征点的状态
    Type type_;

    ///平均视方向??? TODO 
    Vector3d obs_dir_; //!< mean viewing direction, from map point to keyframe
    
    ///什么的最小距离?
    double min_distance_;
    ///什么的最大距离?
    double max_distance_;

    ///参考关键帧  TODO 就是生成这个地图点的关键帧吗
    KeyFramePtr refKF_;

    ///TODO ???  被找到的计数?
    uint64_t found_cunter_;
    ///TODO ?????? 被可视的计数?
    uint64_t visiable_cunter_;

    ///观测线程的线程锁?
    std::mutex mutex_obs_;
    ///TODO ????
    std::mutex mutex_pose_;

};

//typedef std::list<MapPoint::Ptr> MapPoints;

}


#endif