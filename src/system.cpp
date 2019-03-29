/**
 * @file system.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 系统的主框架的实现
 * @version 0.1
 * @date 2019-01-24
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "config.hpp"
#include "system.hpp"
#include "optimizer.hpp"
#include "image_alignment.hpp"
#include "feature_alignment.hpp"
#include "time_tracing.hpp"

namespace ssvo{

///配置文件的路径
std::string Config::file_name_;
///时间记录器
TimeTracing::Ptr sysTrace = nullptr;

//构造函数
System::System(std::string config_file,
               std::string calib_flie) :
    stage_(STAGE_INITALIZE), 
    status_(STATUS_INITAL_RESET),
    last_frame_(nullptr), 
    current_frame_(nullptr), 
    reference_keyframe_(nullptr),
    loopId_(0)      //检测到的发生回环检测的关键帧的id为0?
{
    //step 1 检查文件是否存在
    LOG_ASSERT(!calib_flie.empty()) << "Empty Calibration file input!!!";
    LOG_ASSERT(!config_file.empty()) << "Empty Config file input!!!";
    Config::file_name_ = config_file;

    //step 2 创建摄像头模型
    AbstractCamera::Model model = AbstractCamera::checkCameraModel(calib_flie);
    if(AbstractCamera::Model::PINHOLE == model)
    {
        //创建针孔相机模型
        PinholeCamera::Ptr pinhole_camera = PinholeCamera::create(calib_flie);
        //NOTICE 注意这里的指针模式转换,右侧的是抽象类的指针,左侧是其子类的一个指针
        camera_ = std::static_pointer_cast<AbstractCamera>(pinhole_camera);
    }
    else if(AbstractCamera::Model::ATAN == model)
    {
        //创建atan相机模型
        AtanCamera::Ptr atan_camera = AtanCamera::create(calib_flie);
        camera_ = std::static_pointer_cast<AbstractCamera>(atan_camera);
    }
    else
    {
        LOG(FATAL) << "Error camera model: " << model;
    }

    //step 3 配置图像和特征提取器,跟踪器
    double fps = camera_->fps();
    if(fps < 1.0) fps = 1.0;
    //! image
    const int nlevel = Config::imageNLevel();
    const int width = camera_->width();
    const int height = camera_->height();
    const int image_border = AlignPatch::Size;      //TODO 不太明白为什么要这样子设置。
                                                    //猜测应该是考虑到图像对齐块的大小,需要给图像周围多加上这些大小的空白像素
                                                    //或者是从图像边界上删除这些大小的像素不参与图像对齐
    //! corner detector
    const int grid_size = Config::gridSize();
    const int grid_min_size = Config::gridMinSize();
    const int fast_max_threshold = Config::fastMaxThreshold();
    const int fast_min_threshold = Config::fastMinThreshold();

    
    fast_detector_ = FastDetector::create(
        width, height, 
        image_border, 
        nlevel, 
        grid_size, grid_min_size, 
        fast_max_threshold, fast_min_threshold);
    
    feature_tracker_ = FeatureTracker::create(
        width, height, 
        20,             //TODO 不明白为什么这里给设置成为了20 ?
        image_border, 
        true);          //产生汇报数据 TODO 啥是汇报数据 
    initializer_ = Initializer::create(
        fast_detector_,
        true);
#ifdef SSVO_DBOW_ENABLE

    //step 4 生成词袋模型
    //获取词典保存路径
    std::string voc_dir = Config::DBoWDirectory();
    LOG_ASSERT(!voc_dir.empty()) << "Please check the config file! The DBoW directory is not set!";
    //根据保存的词典创建词袋模型
    DBoW3::Vocabulary* vocabulary = new DBoW3::Vocabulary(voc_dir);
    //接着根据词典创建数据库
    DBoW3::Database* database= new DBoW3::Database(
        *vocabulary,    //词典句柄 
        true,           //使用直接索引来存储特征索引(目前还不是很明白)
        4);             //和向词袋模型中添加图像时, 词典树节点的层数有关

    //step 5 生成局部地图和回环检测器
    mapper_ = LocalMapper::create(
        vocabulary,         //词典
        database,           //数据库
        fast_detector_,     //fast角点探测器句柄
        true, false);       //汇报信息设置

    loop_closure_ = LoopClosure::creat(vocabulary, database);
    //NOTICE 在这里开始了回环检测线程
    loop_closure_->startMainThread();

    mapper_->setLoopCloser(loop_closure_);
    loop_closure_->setLocalMapper(mapper_);         //这个不应该是在回环检测线程创建之前进行吗? TODO 
#else
    mapper_ = LocalMapper::create(fast_detector_, true, false);
#endif
    //step 6 创建深度滤波器
    //设置回调函数句柄
    DepthFilter::Callback depth_fliter_callback = std::bind(
        &LocalMapper::createFeatureFromSeed,        //待绑定函数句柄
        mapper_,                                    //参数1,局部地图句柄 
        std::placeholders::_1);
    depth_filter_ = DepthFilter::create(fast_detector_, depth_fliter_callback, true);
    //step 7 创建可视化窗口
    viewer_ = Viewer::create(mapper_->map_, cv::Size(width, height));

    //NOTICE 建图线程和深度滤波器线程是在这里开启的
    mapper_->startMainThread();
    depth_filter_->startMainThread();

    //每一帧的耗时
    time_ = 1000.0/fps;

    options_.min_kf_disparity = 50;//MIN(Config::imageHeight(), Config::imageWidth())/5;
    options_.min_ref_track_rate = 0.9;


    //! LOG and timer for system;
    //step 7 设置系统的时间计数器和相关日志
    TimeTracing::TraceNames time_names;
    time_names.push_back("total");
    time_names.push_back("processing");
    time_names.push_back("frame_create");
    time_names.push_back("img_align");
    time_names.push_back("feature_reproj");
    time_names.push_back("motion_ba");
    time_names.push_back("light_affine");
    time_names.push_back("per_depth_filter");
    time_names.push_back("finish");

    TimeTracing::TraceNames log_names;
    log_names.push_back("frame_id");
    log_names.push_back("num_feature_reproj");
    log_names.push_back("stage");
    
    string trace_dir = Config::timeTracingDirectory();
    sysTrace.reset(new TimeTracing("ssvo_trace_system", trace_dir, time_names, log_names));
}

//析构
System::~System()
{
    //复位的时候清空各个缓冲器
    sysTrace.reset();

    //停止进程,当前一共具有六个进程
    viewer_->setStop();
    depth_filter_->stopMainThread();
    mapper_->stopMainThread();
    loop_closure_->stopMainThread();
    //这个特殊一些,要等一下
    viewer_->waitForFinish();
}

//处理帧,其实也算上是这个视觉里程计的主循环了吧
void System::process(
    const cv::Mat &image, 
    const double timestamp)
{
    //start timer
    sysTrace->startTimer("total");
    sysTrace->startTimer("frame_create");

    //! get gray image
    //step 1 get gray image
    double t0 = (double)cv::getTickCount();
    rgb_ = image;
    cv::Mat gray = image.clone();
    if(gray.channels() == 3)
        cv::cvtColor(gray, gray, cv::COLOR_RGB2GRAY);

    //step 2 creat current frame
    current_frame_ = Frame::create(gray, timestamp, camera_);
    double t1 = (double)cv::getTickCount();
    LOG(WARNING) << "[System] Frame " << current_frame_->id_ << " create time: " << (t1-t0)/cv::getTickFrequency();
    sysTrace->log("frame_id", current_frame_->id_);
    sysTrace->stopTimer("frame_create");

    //step 3 run processing part depend on stage
    sysTrace->startTimer("processing");
    if(STAGE_NORMAL_FRAME == stage_)
    {
        status_ = tracking();
    }
    else if(STAGE_INITALIZE == stage_)
    {
        status_ = initialize();
    }
    else if(STAGE_RELOCALIZING == stage_)
    {
        status_ = relocalize();
    }
    sysTrace->stopTimer("processing");

    //step 4 operation after a frame
    finishFrame();
}

System::Status System::initialize()
{
    const Initializer::Result result = initializer_->addImage(current_frame_);

    if(result == Initializer::RESET)
        return STATUS_INITAL_RESET;
    else if(result == Initializer::FAILURE || result == Initializer::READY)
        return STATUS_INITAL_PROCESS;

    std::vector<Vector3d> points;
    initializer_->createInitalMap(Config::mapScale());
    mapper_->createInitalMap(initializer_->getReferenceFrame(), current_frame_);

    LOG(WARNING) << "[System] Start two-view BA";

    KeyFrame::Ptr kf0 = mapper_->map_->getKeyFrame(0);
    KeyFrame::Ptr kf1 = mapper_->map_->getKeyFrame(1);

    LOG_ASSERT(kf0 != nullptr && kf1 != nullptr) << "Can not find intial keyframes in map!";

    Optimizer::globleBundleAdjustment(mapper_->map_, 20, 0, true);

    LOG(WARNING) << "[System] End of two-view BA";

    current_frame_->setPose(kf1->pose());
    current_frame_->setRefKeyFrame(kf1);
    reference_keyframe_ = kf1;
    last_keyframe_ = kf1;

    depth_filter_->insertFrame(current_frame_, kf1);

    initializer_->reset();

    return STATUS_INITAL_SUCCEED;
}

System::Status System::tracking()
{
    //首先停止局部地图更新
    std::unique_lock<std::mutex> lock(mapper_->map_->mutex_update_);
    //! loop closure need
    if(loop_closure_->update_finish_ == true)
    {
        //当回环检测的更新过程完成以后再进行下面的操作
        //但是没有太看明白
        KeyFrame::Ptr ref = last_frame_->getRefKeyFrame();
        SE3d Tlr = last_frame_->Tcw()* ref->beforeUpdate_Tcw_.inverse();
        last_frame_->setTcw( Tlr * ref->Tcw() );
        loop_closure_->update_finish_ = false;
    }

    //设置参考关键帧
    current_frame_->setRefKeyFrame(reference_keyframe_);

    //! track seeds
    //深度滤波器开始跟踪当前帧
    depth_filter_->trackFrame(last_frame_, current_frame_);

    // TODO 先验信息怎么设置？[师兄注释]
    //现将当前帧的位姿设置为和上一帧相同
    current_frame_->setPose(last_frame_->pose());
    //! alignment by SE3
    AlignSE3 align;
    sysTrace->startTimer("img_align");
    //当前帧开始和上一帧进行图像对齐
    align.run(last_frame_, current_frame_, Config::alignTopLevel(), Config::alignBottomLevel(), 30, 1e-8);
    sysTrace->stopTimer("img_align");

    //! track local map
    sysTrace->startTimer("feature_reproj");
    //局部地图在当前帧上进行重投影操作
    int matches = feature_tracker_->reprojectLoaclMap(current_frame_);
    sysTrace->stopTimer("feature_reproj");
    sysTrace->log("num_feature_reproj", matches);
    LOG(WARNING) << "[System] Track with " << matches << " points";

    // TODO tracking status [师兄添加]
    //如果重投影所得到的点的数目不符合要求,那么说明本次追踪失败了
    if(matches < Config::minQualityFts())
        return STATUS_TRACKING_BAD;

    //! motion-only BA
    sysTrace->startTimer("motion_ba");
    //运动BA
    Optimizer::motionOnlyBundleAdjustment(current_frame_, false, false, true);
    sysTrace->stopTimer("motion_ba");

    sysTrace->startTimer("per_depth_filter");
    //查看当前帧是否满足创建一个新关键帧的条件
    if(createNewKeyFrame())
    {
        //如果是,那么就现在深度滤波器中插入一帧和参考关键帧
        //这里的插入操作可能和自己想到的不一样
        depth_filter_->insertFrame(current_frame_, reference_keyframe_);
        //然后在建图器中插入参考关键帧 TODO 为什么是这样子的操作?
        mapper_->insertKeyFrame(reference_keyframe_);
    }
    else
    {
        //否则的话就只是插入当前帧
        depth_filter_->insertFrame(current_frame_, nullptr);
    }
    sysTrace->stopTimer("per_depth_filter");

    sysTrace->startTimer("light_affine");
    //TODO 计算仿射矩阵?
    calcLightAffine();
    sysTrace->stopTimer("light_affine");

    //！ save frame pose
    frame_timestamp_buffer_.push_back(current_frame_->timestamp_);
    reference_keyframe_buffer_.push_back(current_frame_->getRefKeyFrame());
    frame_pose_buffer_.push_back(current_frame_->pose());//current_frame_->getRefKeyFrame()->Tcw() * current_frame_->pose());

    return STATUS_TRACKING_GOOD;
}

System::Status System::relocalize()
{
    Corners corners_new;
    Corners corners_old;
    fast_detector_->detect(current_frame_->images(), corners_new, corners_old, Config::minCornersPerKeyFrame());

    reference_keyframe_ = mapper_->relocalizeByDBoW(current_frame_, corners_new);

    if(reference_keyframe_ == nullptr)
        return STATUS_TRACKING_BAD;

    current_frame_->setPose(reference_keyframe_->pose());

    //! alignment by SE3
    AlignSE3 align;
    int matches = align.run(reference_keyframe_, current_frame_, Config::alignTopLevel(), Config::alignBottomLevel(), 30, 1e-8);

    if(matches < 30)
        return STATUS_TRACKING_BAD;

    current_frame_->setRefKeyFrame(reference_keyframe_);
    matches = feature_tracker_->reprojectLoaclMap(current_frame_);

    if(matches < 30)
        return STATUS_TRACKING_BAD;

    Optimizer::motionOnlyBundleAdjustment(current_frame_, false, true, true);

    if(current_frame_->featureNumber() < 30)
        return STATUS_TRACKING_BAD;

    return STATUS_TRACKING_GOOD;
}

void System::calcLightAffine()
{
    std::vector<Feature::Ptr> fts_last = last_frame_->getFeatures();

    const cv::Mat img_last = last_frame_->getImage(0);
    const cv::Mat img_curr = current_frame_->getImage(0).clone() * 1.3;

    const int size = 4;
    const int patch_area = size*size;
    const int N = (int)fts_last.size();
    cv::Mat patch_buffer_last = cv::Mat::zeros(N, patch_area, CV_32FC1);
    cv::Mat patch_buffer_curr = cv::Mat::zeros(N, patch_area, CV_32FC1);

    int count = 0;
    for(int i = 0; i < N; ++i)
    {
        const Feature::Ptr ft_last = fts_last[i];
        const Feature::Ptr ft_curr = current_frame_->getFeatureByMapPoint(ft_last->mpt_);

        if(ft_curr == nullptr)
            continue;

        utils::interpolateMat<uchar, float, size>(img_last, patch_buffer_last.ptr<float>(count), ft_last->px_[0], ft_last->px_[1]);
        utils::interpolateMat<uchar, float, size>(img_curr, patch_buffer_curr.ptr<float>(count), ft_curr->px_[0], ft_curr->px_[1]);

        count++;
    }

    patch_buffer_last.resize(count);
    patch_buffer_curr.resize(count);

    if(count < 20)
    {
        Frame::light_affine_a_ = 1;
        Frame::light_affine_b_ = 0;
        return;
    }

    float a=1;
    float b=0;
    calculateLightAffine(patch_buffer_last, patch_buffer_curr, a, b);
    Frame::light_affine_a_ = a;
    Frame::light_affine_b_ = b;

//    std::cout << "a: " << a << " b: " << b << std::endl;
}

bool System::createNewKeyFrame()
{
    if(mapper_->isRequiredStop())
        return false;

    std::map<KeyFrame::Ptr, int> overlap_kfs = current_frame_->getOverLapKeyFrames();

    std::vector<Feature::Ptr> fts = current_frame_->getFeatures();
    std::map<MapPoint::Ptr, Feature::Ptr> mpt_ft;
    for(const Feature::Ptr &ft : fts)
    {
        mpt_ft.emplace(ft->mpt_, ft);
    }

    KeyFrame::Ptr max_overlap_keyframe;
    int max_overlap = 0;
    for(const auto &olp_kf : overlap_kfs)
    {
        if(olp_kf.second < max_overlap || (olp_kf.second == max_overlap && olp_kf.first->id_ < max_overlap_keyframe->id_))
            continue;

        max_overlap_keyframe = olp_kf.first;
        max_overlap = olp_kf.second;
    }

    //! check distance
    bool c1 = true;
    double median_depth = std::numeric_limits<double>::max();
    double min_depth = std::numeric_limits<double>::max();
    current_frame_->getSceneDepth(median_depth, min_depth);
//    for(const auto &ovlp_kf : overlap_kfs)
//    {
//        SE3d T_cur_from_ref = current_frame_->Tcw() * ovlp_kf.first->pose();
//        Vector3d tran = T_cur_from_ref.translation();
//        double dist1 = tran.dot(tran);
//        double dist2 = 0.1 * (T_cur_from_ref.rotationMatrix() - Matrix3d::Identity()).norm();
//        double dist = dist1 + dist2;
////        std::cout << "d1: " << dist1 << ". d2: " << dist2 << std::endl;
//        if(dist  < 0.10 * median_depth)
//        {
//            c1 = false;
//            break;
//        }
//    }

    SE3d T_cur_from_ref = current_frame_->Tcw() * last_keyframe_->pose();
    Vector3d tran = T_cur_from_ref.translation();
    double dist1 = tran.dot(tran);
    double dist2 = 0.01 * (T_cur_from_ref.rotationMatrix() - Matrix3d::Identity()).norm();
    if(dist1+dist2  < 0.005 * median_depth)
        c1 = false;

    //! check disparity
    std::list<float> disparities;
    const int threahold = int (max_overlap * 0.6);
    for(const auto &ovlp_kf : overlap_kfs)
    {
        if(ovlp_kf.second < threahold)
            continue;

        std::vector<float> disparity;
        disparity.reserve(ovlp_kf.second);
        std::vector<MapPoint::Ptr> mpts = ovlp_kf.first->getMapPoints();
        for(const MapPoint::Ptr &mpt : mpts)
        {
            Feature::Ptr ft_ref = mpt->findObservation(ovlp_kf.first);
            if(ft_ref == nullptr) continue;

            if(!mpt_ft.count(mpt)) continue;
            Feature::Ptr ft_cur = mpt_ft.find(mpt)->second;

            const Vector2d px(ft_ref->px_ - ft_cur->px_);
            disparity.push_back(px.norm());
        }

        std::sort(disparity.begin(), disparity.end());
        float disp = disparity.at(disparity.size()/2);
        disparities.push_back(disp);
    }
    disparities.sort();

    if(!disparities.empty())
        current_frame_->disparity_ = *std::next(disparities.begin(), disparities.size()/2);

    LOG(INFO) << "[System] Max overlap: " << max_overlap << " min disaprity " << disparities.front() << ", median: " << current_frame_->disparity_;

//    int all_features = current_frame_->featureNumber() + current_frame_->seedNumber();
    bool c2 = disparities.front() > options_.min_kf_disparity;
    bool c3 = current_frame_->featureNumber() < reference_keyframe_->featureNumber() * options_.min_ref_track_rate;
//    bool c4 = current_frame_->featureNumber() < reference_keyframe_->featureNumber() * 0.9;

    //! create new keyFrame
    if(c1 && (c2 || c3))
    {
        //! create new keyframe
        KeyFrame::Ptr new_keyframe = KeyFrame::create(current_frame_);
        for(const Feature::Ptr &ft : fts)
        {
            if(ft->mpt_->isBad())
            {
                current_frame_->removeFeature(ft);
                continue;
            }

            ft->mpt_->addObservation(new_keyframe, ft);
            ft->mpt_->updateViewAndDepth();
//            mapper_->addOptimalizeMapPoint(ft->mpt_);
        }
        new_keyframe->updateConnections();
        reference_keyframe_ = new_keyframe;
        last_keyframe_ = new_keyframe;
//        LOG(ERROR) << "C: (" << c1 << ", " << c2 << ", " << c3 << ") cur_n: " << current_frame_->N() << " ck: " << reference_keyframe_->N();
        return true;
    }
        //! change reference keyframe
    else
    {
        if(overlap_kfs[reference_keyframe_] < max_overlap * 0.85)
            reference_keyframe_ = max_overlap_keyframe;
        return false;
    }
}

void System::finishFrame()
{
    sysTrace->startTimer("finish");
    cv::Mat image_show;
//    Stage last_stage = stage_;
    //step 1 根据上一帧的工作内容,更新状态和要显示的位姿等数据
    if(STAGE_NORMAL_FRAME == stage_)
    {
        if(STATUS_TRACKING_BAD == status_)
        {
            stage_ = STAGE_RELOCALIZING;
            current_frame_->setPose(last_frame_->pose());
        }
    }
    else if(STAGE_INITALIZE == stage_)
    {
        if(STATUS_INITAL_SUCCEED == status_)
            stage_ = STAGE_NORMAL_FRAME;
        else if(STATUS_INITAL_RESET == status_)
            initializer_->reset();

        initializer_->drowOpticalFlow(image_show);
    }
    else if(STAGE_RELOCALIZING == stage_)
    {
        if(STATUS_TRACKING_GOOD == status_)
            stage_ = STAGE_NORMAL_FRAME;
        else
            current_frame_->setPose(last_frame_->pose());
    }

    //! update
    //step 2 当前帧变上一帧
    last_frame_ = current_frame_;

    //! display
    //step 3 绘制当前帧
    viewer_->setCurrentFrame(current_frame_, image_show);

    //step 4 时间记录器,日志,轨迹记录等操作
    sysTrace->log("stage", stage_);
    sysTrace->stopTimer("finish");
    sysTrace->stopTimer("total");
    const double time = sysTrace->getTimer("total");
    LOG(WARNING) << "[System] Finish Current Frame with Stage: " << stage_ << ", total time: " << time;

    sysTrace->writeToFile();

}

void System::saveTrajectoryTUM(const std::string &file_name)
{
    std::ofstream f;
    f.open(file_name.c_str());
    f << std::fixed;

    std::list<double>::iterator frame_timestamp_ptr = frame_timestamp_buffer_.begin();
    std::list<Sophus::SE3d>::iterator frame_pose_ptr = frame_pose_buffer_.begin();
    std::list<KeyFrame::Ptr>::iterator reference_keyframe_ptr = reference_keyframe_buffer_.begin();
    const std::list<double>::iterator frame_timestamp = frame_timestamp_buffer_.end();
    for(; frame_timestamp_ptr!= frame_timestamp; frame_timestamp_ptr++, frame_pose_ptr++, reference_keyframe_ptr++)
    {
        Sophus::SE3d frame_pose = (*frame_pose_ptr);//(*reference_keyframe_ptr)->Twc() * (*frame_pose_ptr);
        Vector3d t = frame_pose.translation();
        Quaterniond q = frame_pose.unit_quaternion();

        f << std::setprecision(6) << *frame_timestamp_ptr << " "
          << std::setprecision(9) << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    f.close();
    LOG(INFO) << " trajectory saved!";

    std::string KFfilename = "KF" + file_name;

    f.open(KFfilename.c_str());
    f << std::fixed;

    std::vector<KeyFrame::Ptr> kfs = mapper_->map_->getAllKeyFrames();
    std::sort(kfs.begin(),kfs.end(),[](KeyFrame::Ptr kf1,KeyFrame::Ptr kf2)->bool{ return kf1->timestamp_<kf2->timestamp_;});

    for(auto kf:kfs)
    {
        Sophus::SE3d frame_pose = kf->pose();//(*reference_keyframe_ptr)->Twc() * (*frame_pose_ptr);
        Vector3d t = frame_pose.translation();
        Quaterniond q = frame_pose.unit_quaternion();

        f << std::setprecision(6) << kf->timestamp_ << " "
          << std::setprecision(9) << t[0] << " " << t[1] << " " << t[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    f.close();
    LOG(INFO) << " KFtrajectory saved!";


}

}

