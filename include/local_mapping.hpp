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

class LoopClosure;

class LocalMapper : public noncopyable
{
public:

    typedef std::shared_ptr<LocalMapper> Ptr;

    void createInitalMap(const Frame::Ptr &frame_ref, const Frame::Ptr &frame_cur);

    void insertKeyFrame(const KeyFrame::Ptr &keyframe);

    void startMainThread();

    void stopMainThread();

    void addOptimalizeMapPoint(const MapPoint::Ptr &mpt);

    int refineMapPoints(const int max_optimalize_num = -1, const double outlier_thr = 2.0/480.0);

    void createFeatureFromSeed(const Seed::Ptr &seed);

    KeyFrame::Ptr relocalizeByDBoW(const Frame::Ptr &frame, const Corners &corners);

    static LocalMapper::Ptr create(const FastDetector::Ptr fast, bool report = false, bool verbose = false)
    { return LocalMapper::Ptr(new LocalMapper(fast, report, verbose));}

#ifdef SSVO_DBOW_ENABLE
    static LocalMapper::Ptr create(DBoW3::Vocabulary* vocabulary, DBoW3::Database* database, const FastDetector::Ptr fast, bool report = false, bool verbose = false)
    { return LocalMapper::Ptr(new LocalMapper(vocabulary, database, fast, report, verbose));}
        
    void setLoopCloser(std::shared_ptr<LoopClosure> loop_closure);
#endif

    void setStop();

    bool isRequiredStop();

    void release();

    bool finish_once();
private:

    LocalMapper(const FastDetector::Ptr fast, bool report, bool verbose);

#ifdef SSVO_DBOW_ENABLE
    LocalMapper(DBoW3::Vocabulary* vocabulary, DBoW3::Database* database,const FastDetector::Ptr fast, bool report, bool verbose);

    std::shared_ptr<LoopClosure> loop_closure_;
#endif
    void run();

    KeyFrame::Ptr checkNewKeyFrame();

    void finishLastKeyFrame();

    int createFeatureFromSeedFeature(const KeyFrame::Ptr &keyframe);

    int createFeatureFromLocalMap(const KeyFrame::Ptr &keyframe, const int num = 5);

    void checkCulling(const KeyFrame::Ptr &keyframe);

    void addToDatabase(const KeyFrame::Ptr &keyframe);

public:

    Map::Ptr map_;

private:

    struct Option{
        double min_disparity;
        int min_redundant_observations;
        int max_features;
        int num_reproject_kfs;
        int num_local_ba_kfs;
        int min_local_ba_connected_fts;
        int num_align_iter;
        double max_align_epsilon;
        double max_align_error2;
        double min_found_ratio_;
    } options_;

    FastDetector::Ptr fast_detector_;

    BRIEF::Ptr brief_;

    std::deque<KeyFrame::Ptr> keyframes_buffer_;
    KeyFrame::Ptr keyframe_last_;

#ifdef SSVO_DBOW_ENABLE
    DBoW3::Vocabulary* vocabulary_;
    DBoW3::Database* database_;

#endif

    const bool report_;
    const bool verbose_;

    std::shared_ptr<std::thread> mapping_thread_;

    std::list<MapPoint::Ptr> optimalize_candidate_mpts_;

    bool stop_require_;
    bool finish_once_;
    std::mutex mutex_stop_;
    std::mutex mutex_keyframe_;
    std::mutex mutex_optimalize_mpts_;
    std::condition_variable cond_process_;

};

}

#endif //_SSVO_LOCAL_MAPPING_HPP_
