#include "config.hpp"
#include "system.hpp"
#include "optimizer.hpp"
#include "image_alignment.hpp"
#include "feature_alignment.hpp"
#include "time_tracing.hpp"

namespace ssvo{

std::string Config::file_name_;

TimeTracing::Ptr sysTrace = nullptr;

System::System(std::string config_file, std::string calib_flie) :
    stage_(STAGE_INITALIZE), status_(STATUS_INITAL_RESET),
    last_frame_(nullptr), current_frame_(nullptr), reference_keyframe_(nullptr),loopId_(0)
{
    LOG_ASSERT(!calib_flie.empty()) << "Empty Calibration file input!!!";
    LOG_ASSERT(!config_file.empty()) << "Empty Config file input!!!";
    Config::file_name_ = config_file;

    AbstractCamera::Model model = AbstractCamera::checkCameraModel(calib_flie);
    if(AbstractCamera::Model::PINHOLE == model)
    {
        PinholeCamera::Ptr pinhole_camera = PinholeCamera::create(calib_flie);
        camera_ = std::static_pointer_cast<AbstractCamera>(pinhole_camera);
    }
    else if(AbstractCamera::Model::ATAN == model)
    {
        AtanCamera::Ptr atan_camera = AtanCamera::create(calib_flie);
        camera_ = std::static_pointer_cast<AbstractCamera>(atan_camera);
    }
    else
    {
        LOG(FATAL) << "Error camera model: " << model;
    }

    double fps = camera_->fps();
    if(fps < 1.0) fps = 1.0;
    //! image
    const int nlevel = Config::imageNLevel();
    const int width = camera_->width();
    const int height = camera_->height();
    const int image_border = AlignPatch::Size;
    //! corner detector
    const int grid_size = Config::gridSize();
    const int grid_min_size = Config::gridMinSize();
    const int fast_max_threshold = Config::fastMaxThreshold();
    const int fast_min_threshold = Config::fastMinThreshold();

    fast_detector_ = FastDetector::create(width, height, image_border, nlevel, grid_size, grid_min_size, fast_max_threshold, fast_min_threshold);
    feature_tracker_ = FeatureTracker::create(width, height, 20, image_border, true);
    initializer_ = Initializer::create(fast_detector_, true);
#ifdef SSVO_DBOW_ENABLE
    std::string voc_dir = Config::DBoWDirectory();

    LOG_ASSERT(!voc_dir.empty()) << "Please check the config file! The DBoW directory is not set!";
    DBoW3::Vocabulary* vocabulary = new DBoW3::Vocabulary(voc_dir);
    DBoW3::Database* database= new DBoW3::Database(*vocabulary, true, 4);

    mapper_ = LocalMapper::create(vocabulary, database, fast_detector_, true, false);

    loop_closure_ = LoopClosure::creat(vocabulary, database);
    loop_closure_->startMainThread();

    mapper_->setLoopCloser(loop_closure_);
    loop_closure_->setLocalMapper(mapper_);
#else
    mapper_ = LocalMapper::create(fast_detector_, true, false);
#endif
    DepthFilter::Callback depth_fliter_callback = std::bind(&LocalMapper::createFeatureFromSeed, mapper_, std::placeholders::_1);
    depth_filter_ = DepthFilter::create(fast_detector_, depth_fliter_callback, true);
    viewer_ = Viewer::create(mapper_->map_, cv::Size(width, height));

    mapper_->startMainThread();
    depth_filter_->startMainThread();

    time_ = 1000.0/fps;

    options_.min_kf_disparity = 50;//MIN(Config::imageHeight(), Config::imageWidth())/5;
    options_.min_ref_track_rate = 0.9;


    //! LOG and timer for system;
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

System::~System()
{
    sysTrace.reset();

    viewer_->setStop();
    depth_filter_->stopMainThread();
    mapper_->stopMainThread();
    loop_closure_->stopMainThread();

    viewer_->waitForFinish();
}

void System::process(const cv::Mat &image, const double timestamp)
{
    sysTrace->startTimer("total");
    sysTrace->startTimer("frame_create");
    //! get gray image
    double t0 = (double)cv::getTickCount();
    rgb_ = image;
    cv::Mat gray = image.clone();
    if(gray.channels() == 3)
        cv::cvtColor(gray, gray, cv::COLOR_RGB2GRAY);

    current_frame_ = Frame::create(gray, timestamp, camera_);
    double t1 = (double)cv::getTickCount();
    LOG(WARNING) << "[System] Frame " << current_frame_->id_ << " create time: " << (t1-t0)/cv::getTickFrequency();
    sysTrace->log("frame_id", current_frame_->id_);
    sysTrace->stopTimer("frame_create");

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
    std::unique_lock<std::mutex> lock(mapper_->map_->mutex_update_);
    //! loop closure need
    if(loop_closure_->update_finish_ == true)
    {
        KeyFrame::Ptr ref = last_frame_->getRefKeyFrame();
        SE3d Tlr = last_frame_->Tcw()* ref->beforeUpdate_Tcw_.inverse();
        last_frame_->setTcw( Tlr * ref->Tcw() );
        loop_closure_->update_finish_ = false;
    }

    current_frame_->setRefKeyFrame(reference_keyframe_);

    //! track seeds
    depth_filter_->trackFrame(last_frame_, current_frame_);

    // TODO 先验信息怎么设置？
    current_frame_->setPose(last_frame_->pose());
    //! alignment by SE3
    AlignSE3 align;
    sysTrace->startTimer("img_align");
    align.run(last_frame_, current_frame_, Config::alignTopLevel(), Config::alignBottomLevel(), 30, 1e-8);
    sysTrace->stopTimer("img_align");

    //! track local map
    sysTrace->startTimer("feature_reproj");
    int matches = feature_tracker_->reprojectLoaclMap(current_frame_);
    sysTrace->stopTimer("feature_reproj");
    sysTrace->log("num_feature_reproj", matches);
    LOG(WARNING) << "[System] Track with " << matches << " points";

    // TODO tracking status
    if(matches < Config::minQualityFts())
        return STATUS_TRACKING_BAD;

    //! motion-only BA
    sysTrace->startTimer("motion_ba");
    Optimizer::motionOnlyBundleAdjustment(current_frame_, false, false, true);
    sysTrace->stopTimer("motion_ba");

    sysTrace->startTimer("per_depth_filter");
    if(createNewKeyFrame())
    {
        depth_filter_->insertFrame(current_frame_, reference_keyframe_);
        mapper_->insertKeyFrame(reference_keyframe_);
    }
    else
    {
        depth_filter_->insertFrame(current_frame_, nullptr);
    }
    sysTrace->stopTimer("per_depth_filter");

    sysTrace->startTimer("light_affine");
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
    last_frame_ = current_frame_;

    //! display
    viewer_->setCurrentFrame(current_frame_, image_show);

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

