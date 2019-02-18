/**
 * @file viewer.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 绘制可视化的查看器
 * @version 0.1
 * @date 2019-02-18
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "viewer.hpp"

namespace ssvo{

//可视化查看器的创建
Viewer::Viewer(const Map::Ptr &map,     //地图句柄
               cv::Size image_size) :   //图像大小(指的是相机拍摄到的图像大小)
    map_(map),                  
    image_size_(image_size), 
    required_stop_(false), 
    is_finished_(false)
{
    //一些绘图属性设置
    map_point_size = 3;
    key_frame_size = 0.05;
    key_frame_line_width= 2;
    key_frame_graph_line_width = 1;

    //创建可视化查看器进程
    pongolin_thread_ = std::make_shared<std::thread>(std::bind(&Viewer::run, this));
}


void Viewer::setStop()
{
    std::lock_guard<std::mutex> lock(mutex_stop_);
    required_stop_ = true;
}

bool Viewer::isRequiredStop()
{
    std::lock_guard<std::mutex> lock(mutex_stop_);
    return required_stop_;
}

bool Viewer::waitForFinish()
{
    if(!isRequiredStop())
        setStop();

    if(pongolin_thread_->joinable())
        pongolin_thread_->join();
    
    return true;
}

//查看器进程主函数
void Viewer::run()
{
    const int WIN_WIDTH = 1280;
    const int WIN_HEIGHT = 720;
    const int UI_WIDTH = 160;
    const int UI_HEIGHT = 200;      //NOTICE 
    //按比例缩放
    const int IMAGE_WIDTH = UI_HEIGHT *image_size_.width/image_size_.height;
    const int IMAGE_HEIGHT = UI_HEIGHT;

    const string win_name = "SSVO Viewer";
    pangolin::CreateWindowAndBind(win_name, WIN_WIDTH, WIN_HEIGHT);
    glEnable(GL_DEPTH_TEST);

    //颜色混合
    glEnable(GL_BLEND);
    //混合因子设置,参考[http://blog.chinaunix.net/uid-20622737-id-2850251.html]
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu")
        .SetBounds(pangolin::Attach::Pix(WIN_HEIGHT-UI_HEIGHT),     //底部
                   1.0,                                             //顶部
                   0.0,                                             //左
                   pangolin::Attach::Pix(UI_WIDTH));                //右
    
    //都是一些单选按钮
    pangolin::Var<bool> menu_follow_camera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menu_show_trajectory("menu.Show Trajectory", true, true);
    pangolin::Var<bool> menu_show_keyframe("menu.Show KeyFrame", true, true);
    pangolin::Var<bool> menu_show_keyframetraj("menu.Show KFtraj", true,true);
    pangolin::Var<bool> menu_show_connections("menu.Connections", true, true);
    pangolin::Var<bool> menu_show_current_connections("menu.Connections_cur", true, true);

    //TODO 绘制的轨迹的持续长度?
    const int trajectory_duration_max = 10000;
    pangolin::Var<int> settings_trajectory_duration("menu.Traj Duration",1000, 1, trajectory_duration_max,false);

    //其实师兄没有必要再设这些个变量的
    bool following_camera = true;
    bool show_trajectory = true;
    bool show_keyframe = true;
    bool show_keyframetraj = true;
    bool show_connections = true;
    bool show_current_connections = true;
    int trajectory_duration = -1;

    // Define Projection and initial ModelView matrix
    //设置空间场景,设置摄像机
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(WIN_WIDTH, WIN_HEIGHT, 500, 500, WIN_WIDTH/2, WIN_HEIGHT/2, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -1, -0.5, 0, 0, 0, 0, -1, 0));

    // Create Interactive View in window
    //生成摄像机视图(也就是三维视图)
    pangolin::View& camera_viewer = pangolin::Display("Camera")
        .SetBounds(
            0.0,
            1.0,
            pangolin::Attach::Pix(UI_WIDTH),                //注意师兄这里是让生成的摄像机视图紧紧贴着panel的边上的
            1.0, 
            (-1.0*WIN_WIDTH)/WIN_HEIGHT)                    //同样的这里也设计了纵横比
        .SetHandler(new pangolin::Handler3D(s_cam));
    //生成图像视图
    pangolin::View& image_viewer = pangolin::Display("Image")
        .SetBounds(pangolin::Attach::Pix(WIN_HEIGHT-IMAGE_HEIGHT),  //顶部到底部的距离
                   1, 
                   pangolin::Attach::Pix(UI_WIDTH),                 //左侧
                   pangolin::Attach::Pix(UI_WIDTH+IMAGE_WIDTH), 
                   (-1.0*image_size_.width/image_size_.height))
        .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::GlTexture imageTexture(image_size_.width, image_size_.height, GL_RGB, false, 0, GL_BGR,GL_UNSIGNED_BYTE);

    //在线程的主循环中多了一个判断是否有停止条件的标志位的检测
    while(!pangolin::ShouldQuit() && !isRequiredStop())
    {
        
        Frame::Ptr frame;
        cv::Mat image;
        {
            //锁住...帧的线程? TODO 并从其中获取帧对象和图像对象
            std::lock_guard<std::mutex> lock(mutex_frame_);
            frame = frame_;
            image = image_;
        }

        // Clear screen and activate view to render into
        //清空
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        camera_viewer.Activate(s_cam);
        //背景颜色
        glClearColor(1.0f,1.0f,1.0f,1.0f);


        //! update
        //更新按钮状态
        following_camera = menu_follow_camera.Get();
        show_trajectory = menu_show_trajectory.Get();
        show_keyframe = menu_show_keyframe.Get();
        show_keyframetraj = menu_show_keyframetraj.Get();
        show_connections = menu_show_connections.Get();
        show_current_connections = menu_show_current_connections.Get();

        trajectory_duration = settings_trajectory_duration.Get();
        if(trajectory_duration == trajectory_duration_max) trajectory_duration = -1;

        //进行跟踪摄像头的操作
        if(following_camera)
        {
            pangolin::OpenGlMatrix camera_pose;
            //NOTICE 目前不是很明白 GLprecision 是什么意思;不过也没有关系,因为下面的这个T根本就没有使用到
            Eigen::Map<Matrix<pangolin::GLprecision, 4, 4> > T(camera_pose.m);
            if(frame)
            {
                T = frame->pose().matrix();
            }
            else{
                T.setIdentity();
            }

            //s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, -1, -0.5, 0, 0, 0, 0, -1, 0));
            s_cam.Follow(camera_pose);
            following_camera = true;
        }

        //师兄竟然还绘制了坐标轴了
        pangolin::glDrawAxis(0.1);

        //绘制地图点
        drawMapPoints(map_, frame);

        //绘制关键帧
        if(show_keyframe)
        {
            KeyFrame::Ptr reference = frame ? frame->getRefKeyFrame() : nullptr;
            drawKeyFrames(map_, reference, show_connections, show_current_connections);
        }

        //绘制轨迹
        if(show_trajectory)
            drawTrajectory(trajectory_duration);

        //绘制关键帧轨迹
        if(show_keyframetraj)
            drawKfTraj(map_);

        //绘制帧
        if(frame)
            drawCurrentFrame(frame->pose().matrix(), cv::Scalar(0.0, 0.0, 1.0));

        //绘制图像上的追踪点
        if(image.empty() && frame != nullptr)
            drawTrackedPoints(frame, image);

        //绘制当前图像,但是这个是使用OpenCV的方式来绘制的图像
        drawCurrentImage(imageTexture, image);

        //这里才是使用Pangolin的方式来绘制图像
        image_viewer.Activate();
        glColor3f(1.0,1.0,1.0);
        imageTexture.RenderToViewportFlipY();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    //NOTICE 为了严谨,最后要加上这个
    pangolin::DestroyWindow(win_name);
}

void Viewer::setCurrentFrame(const Frame::Ptr &frame, const cv::Mat image)
{
    std::lock_guard<std::mutex> lock(mutex_frame_);
    frame_ = frame;
    image_ = image;
    if(frame_)
        frame_trajectory_.push_back(frame_->pose().translation());
}

//绘制关键帧
void Viewer::drawKeyFrames(Map::Ptr &map, KeyFrame::Ptr &reference, 
    bool show_connections,  //是否绘制共视关系
    bool show_current)      //是否绘制和当前关键帧有共视关系的帧(或者应当说是特别标记出来)
{
    std::vector<KeyFrame::Ptr> kfs = map->getAllKeyFrames();

    std::set<KeyFrame::Ptr> loacl_kfs;

    //获取和当前关键帧有共视关系的帧
    if(show_current)
    {
        if(reference != nullptr)
        {
            loacl_kfs = reference->getConnectedKeyFrames();
            loacl_kfs.insert(reference);
        }
    }

    //绘制关键帧
    for(const KeyFrame::Ptr &kf : kfs)
    {
        SE3d pose = kf->pose();
        //通过查看是否在集合中来调用不同的绘制颜色,感觉比较巧妙
        if(loacl_kfs.count(kf))
            drawCamera(pose.matrix(), cv::Scalar(0.0, 0.5, 1.0));
        else
            drawCamera(pose.matrix(), cv::Scalar(0.0, 1.0, 0.2));
    }


    if(!show_connections)
        return;

    //下面开始绘制共视关系
    glLineWidth(key_frame_graph_line_width);
    glColor4f(0.0f,1.0f,0.0f,0.6f);
    glBegin(GL_LINES);
    //遍历目前系统中的所有关键帧
    for(const KeyFrame::Ptr &kf : kfs)
    {
        //获得当前遍历到的关键帧的相机光心位置
        Vector3f O1 = kf->pose().translation().cast<float>();
        //遍历和这个关键帧具有共视关系的所有关键帧
        const std::set<KeyFrame::Ptr> conect_kfs = kf->getConnectedKeyFrames();
        for(const KeyFrame::Ptr &ckf : conect_kfs)
        {
            //这里还只是绘制比当前遍历到的关键帧生成要晚的关键帧的共视关系
            if(ckf->id_ < kf->id_)
                continue;
            //HERE

            Vector3f O2 = ckf->pose().translation().cast<float>();
            glVertex3f(O1[0], O1[1], O1[2]);
            glVertex3f(O2[0], O2[1], O2[2]);
        }
    }
    glEnd();

}

void Viewer::drawCurrentFrame(const Matrix4d &pose, cv::Scalar color)
{
    drawCamera(pose.matrix(), color);
}

//注意,这个是使用OpenCV方式来绘制图像
void Viewer::drawCurrentImage(pangolin::GlTexture &gl_texture, cv::Mat &image)
{
    if(image.empty())
        return;

    if(image.type() == CV_8UC1)
        cv::cvtColor(image, image, CV_GRAY2RGB);
    gl_texture.Upload(image.data, GL_RGB, GL_UNSIGNED_BYTE);
    cv::imshow("SSVO Current Image", image);
    //NOTICE 注意到,师兄应该也是注意到了我之前发现的问题,所以留出了刷新的时间
    cv::waitKey(1);
}

//绘制地图点
void Viewer::drawMapPoints(Map::Ptr &map, Frame::Ptr &frame)
{
    //获取地图点和特征点的匹配关系
    std::unordered_map<MapPoint::Ptr, Feature::Ptr> obs_mpts;
    if(frame)
        obs_mpts = frame->features();

    //获取地图点集合
    std::vector<MapPoint::Ptr> mpts = map->getAllMapPoints();

    //开始绘制地图点
    glPointSize(map_point_size);
    glBegin(GL_POINTS);
    for(const MapPoint::Ptr &mpt : mpts)
    {
        //别看变量名是pose,但是实际上就是地图点的位置
        Vector3d pose = mpt->pose();
        //如果看到了绘制成这个偏红的颜色
        if(obs_mpts.count(mpt))
            glColor3f(1.0,0.0,0.3);
//        else if(mpt->observations() == 1)
//             glColor3f(0.0,0.0,0.0);
        else
            //如果当前没有追踪到,就绘制成为灰色
            glColor3f(0.5,0.5,0.5);
//        float rate = (float)mpt->getFoundRatio();
//        glColor3f((1-rate)*rate, 0, rate*rate);
        glVertex3f(pose[0], pose[1], pose[2]);
    }
    glEnd();
}

void Viewer::drawCamera(const Matrix4d &pose, cv::Scalar color)
{
    const float w = key_frame_size ;
    const float h = key_frame_size * 0.57f;
    const float z = key_frame_size * 0.6f;

    glPushMatrix();

    //! col major
    glMultMatrixd(pose.data());

    glLineWidth(key_frame_line_width);
    glColor3f((GLfloat)color[0], (GLfloat)color[1], (GLfloat)color[2]);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}

void Viewer::drawTrackedPoints(const Frame::Ptr &frame, cv::Mat &dst)
{
    //! draw features
    const cv::Mat src = frame->getImage(0);
    std::vector<Feature::Ptr> fts = frame->getFeatures();
    cv::cvtColor(src, dst, CV_GRAY2RGB);
    int font_face = 1;
    double font_scale = 0.5;
    for(const Feature::Ptr &ft : fts)
    {
        Vector2d ft_px = ft->px_;
        cv::Point2f px(ft_px[0], ft_px[1]);
        cv::Scalar color(0, 255, 0);
        cv::circle(dst, px, 2, color, -1);

        // 这里就是在点上显示文本的数据
        string id_str = std::to_string((frame->Tcw()*ft->mpt_->pose()).norm());//ft->mpt_->getFoundRatio());//
        cv::putText(dst, id_str, px-cv::Point2f(1,1), font_face, font_scale, color);
    }

    //! draw seeds
    std::vector<Feature::Ptr> seed_fts = frame->getSeeds();
    for(const Feature::Ptr &ft : seed_fts)
    {
        Seed::Ptr seed = ft->seed_;
        cv::Point2f px(ft->px_[0], ft->px_[1]);
        double convergence = 0;
        double scale = MIN(convergence, 256.0) / 256.0;
        cv::Scalar color(255*scale, 0, 255*(1-scale));
        cv::circle(dst, px, 2, color, -1);

//        string id_str = std::to_string();
//        cv::putText(dst, id_str, px-cv::Point2f(1,1), font_face, font_scale, color);
    }

}

void Viewer::drawTrajectory(int frame_num)
{
    std::lock_guard<std::mutex> lock(mutex_frame_);
    float color[3] = {1,0,0};
    glColor3f(color[0],color[1],color[2]);
    glLineWidth(2);

    glBegin(GL_LINE_STRIP);

    size_t frame_count_max = frame_num == -1 ? frame_trajectory_.size() : static_cast<size_t>(frame_num);
    size_t frame_count = 0;
    for(auto itr = frame_trajectory_.rbegin(); itr != frame_trajectory_.rend() && frame_count < frame_count_max; itr++, frame_count++)
    {
        glVertex3f((float)(*itr)[0], (float)(*itr)[1], (float)(*itr)[2]);
    }
    glEnd();
}

void Viewer::drawKfTraj(Map::Ptr &map)
{
    std::lock_guard<std::mutex> lock(mutex_frame_);
    float color[3] = {0,0,1};
    glColor3f(color[0],color[1],color[2]);
    glLineWidth(2);

    glBegin(GL_LINE_STRIP);


    std::vector<KeyFrame::Ptr> kfs = map->getAllKeyFrames();

    std::sort(kfs.begin(),kfs.end(),[](KeyFrame::Ptr a,KeyFrame::Ptr b)->bool{ return a->id_>b->id_;});

    for(const KeyFrame::Ptr &kf : kfs)
    {
        Vector3f O1 = kf->pose().translation().cast<float>();
        glVertex3f((float)O1[0], (float)O1[1], (float)O1[2]);

    }
    glEnd();

}

}
