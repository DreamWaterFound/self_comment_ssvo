/**
 * @file viewer.hpp
 * @author guoqing (1337841346@qq.com)
 * @brief 可视化部分
 * @version 0.1
 * @date 2019-01-22
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _SSVO_VIEWER_HPP_
#define _SSVO_VIEWER_HPP_

#include <pangolin/pangolin.h>
#include "global.hpp"
#include "config.hpp"
#include "map.hpp"

namespace ssvo {

/**
 * @brief 可视化的查看器
 * 
 */
class Viewer : public noncopyable
{
public:
    ///管理本类对象的指针
    typedef std::shared_ptr<Viewer> Ptr;
    /**
     * @brief 查看器主线程
     * 
     */
    void run();
    /**
     * @brief 设置停止查看器线程
     * 
     */
    void setStop();
    /**
     * @brief 等待....完成当前操作? TODO
     * 
     * @return true 
     * @return false 
     */
    bool waitForFinish();
    /**
     * @brief 设置当前的帧
     * 
     * @param[in] frame 当前帧
     * @param[in] image 当前帧的图像
     */
    void setCurrentFrame(const Frame::Ptr &frame, const cv::Mat image = cv::Mat());
    /**
     * @brief 构造函数
     * 
     * @param[in] map           地图
     * @param[in] image_size    图像的显示大小
     * @return Viewer::Ptr      实例指针
     */
    static Viewer::Ptr create(const Map::Ptr &map, cv::Size image_size){ return Viewer::Ptr(new Viewer(map, image_size));}

private:
    /**
     * @brief 构造函数
     * 
     * @param[in] map           地图
     * @param[in] image_size    图像的显示大小
     */
    Viewer(const Map::Ptr &map, cv::Size image_size);
    /**
     * @brief 检查是否请求当前线程停止工作
     * 
     * @return true 
     * @return false 
     */
    bool isRequiredStop();
    /**
     * @brief 设置啥啊? TODO 
     * 
     */
    void setFinished();
    /**
     * @brief 绘制点
     * 
     * @param[in] map       地图句柄 
     * @param[in] frame     帧 TODO 这里的帧是做什么用的
     */
    void drawMapPoints(Map::Ptr &map, Frame::Ptr &frame);
    /**
     * @brief 绘制相机
     * 
     * @param[in] pose      相机的位姿 
     * @param[in] color     相机的颜色
     */
    void drawCamera(const Matrix4d &pose, cv::Scalar color);
    /**
     * @brief 绘制关键帧
     * 
     * @param[in] map               地图句柄
     * @param[in] reference         绘制过程中使用的参考关键帧
     * @param[in] show_connections  是否显示关键帧之间的共视关系
     * @param[in] show_current      TODO ????　这个是啥啊
     */
    void drawKeyFrames(Map::Ptr &map, KeyFrame::Ptr &reference, bool show_connections=false, bool show_current=false);
    /**
     * @brief 绘制当前帧
     * 
     * @param[in] pose  当前帧的位置
     * @param[in] color 当前帧的颜色
     */
    void drawCurrentFrame(const Matrix4d &pose, cv::Scalar color);
    /**
     * @brief 绘制当前图像
     * 
     * @param[in] gl_texture TODO ????
     * @param[in] image      图像内容
     */
    void drawCurrentImage(pangolin::GlTexture& gl_texture, cv::Mat &image);
    /**
     * @brief 绘制追踪到的点(在图像的显示窗口中)
     * 
     * @param[in] frame     哪一帧?
     * @param[out] dst       画布
     */
    void drawTrackedPoints(const Frame::Ptr &frame, cv::Mat &dst);
    /**
     * @brief 绘制轨迹
     * 
     * @param[in] frame_num 指定帧数? TODO 
     */
    void drawTrajectory(int frame_num = -1);
    /**
     * @brief 绘制关键帧的轨迹
     * 
     * @param[in] map 地图句柄
     */
    void drawKfTraj(Map::Ptr &map);

private:
    ///查看器的线程
    std::shared_ptr<std::thread> pongolin_thread_;
    ///地图句柄
    Map::Ptr map_;
    ///帧句柄
    Frame::Ptr frame_;
    ///图像
    cv::Mat image_;
    ///图像的尺寸
    cv::Size image_size_;

    ///帧的轨迹
    std::list<Vector3d, aligned_allocator<Vector3d> > frame_trajectory_;
    ///关键帧的轨迹
    std::list<Vector3d, aligned_allocator<Vector3d> > kf_trajectory_;

    ///绘制的地图点的大小
    float map_point_size;
    ///绘制的关键帧的大小 
    float key_frame_size;
    ///绘制的关键帧的线的宽度
    float key_frame_line_width;
    ///绘制的关键帧的链接关系的线的宽度
    float key_frame_graph_line_width;

    ///是否有停止当前线程的请求标志
    bool required_stop_;
    ///TODO 
    bool is_finished_;

    ///线程锁
    std::mutex mutex_frame_;
    ///线程锁
    std::mutex mutex_stop_;
}; //class viewer

} //namespace ssvo

#endif //_SSVO_VIEWER_HPP_
