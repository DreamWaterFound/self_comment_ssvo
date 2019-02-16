/**
 * @file monoVO_tum.cpp
 * @author guoqing (1337841346@qq.com)
 * @brief 在TUM RGBD 数据集上读取数据
 * @version 0.1
 * @date 2019-02-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "system.hpp"
#include "dataset.hpp"
#include "time_tracing.hpp"

using namespace ssvo;

int main(int argc, char *argv[])
{
    //日志支持
    google::InitGoogleLogging(argv[0]);
    LOG_ASSERT(argc == 5) << "\n Usage : ./monoVO_tum config_file calib_file dataset_path association_file ";

    //生成视觉里程计对象
    System vo(argv[1], argv[2]);

    //数据集读取器
    TUMDataReader dataset(argv[3],argv[4]);

    //定时器
    ssvo::Timer<std::micro> timer;
    const size_t N = dataset.N;
    for(size_t i = 0; i < N; i++)
    {
        string rgb_,depth_;
        double timestamp;
        //通过索引读取彩色图像、深度图像的路径和时间戳
        if(!dataset.readItemByIndex(i,rgb_,depth_,timestamp))
            std::cerr<<"NO image to load"<<std::endl;

        LOG(INFO) << "=== Load Image " << i << ": " << rgb_ << ", time: " << std::fixed <<std::setprecision(7)<< timestamp << std::endl;
        //读入图像,并且对图像的合法性进行检查
        cv::Mat image = cv::imread(rgb_, CV_LOAD_IMAGE_UNCHANGED);
        if(image.empty())
            continue;

        //调用VO进行追踪
        timer.start();
        vo.process(image, timestamp);
        timer.stop();
        //计算耗时
        double time_process = timer.duration();
        //计算需要等待的时间
        double time_wait = 0;
        if(i < N -1)
            time_wait = (dataset.timestamps_[i+1] - dataset.timestamps_[i])*1e6;
        else
            time_wait = (dataset.timestamps_[i] - dataset.timestamps_[i-1])*1e6;

        //当然,只有在需要等待的时候,才会待着等
        if(time_process < time_wait)
            std::this_thread::sleep_for(std::chrono::microseconds((int)(time_wait - time_process)));


    }

    //保存轨迹
    vo.saveTrajectoryTUM("trajectory.txt");
    //按任意键停止
    getchar();

    return 0;
}