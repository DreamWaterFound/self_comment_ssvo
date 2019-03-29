#ifndef _SSVO_TIME_TRACING_HPP_
#define _SSVO_TIME_TRACING_HPP_

#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <list>
#include <unordered_map>

//REVIEW 本文件中涉及到的源代码没有看

namespace ssvo
{

/**
 * @brief 计时用的
 * 
 * @tparam T   计时单位(ms or s)
 */
template<typename T>
class Timer
{
public:

    /**
     * @brief 开始计时
     * 
     */
    inline void start()
    {
        start_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief 停止计时
     * 
     * @return double 
     */
    inline double stop()
    {
        duration_ = std::chrono::steady_clock::now() - start_;
        return duration_.count();
    }

    /**
     * @brief 定时器复位
     * 
     */
    inline void reset()
    {
        duration_ = std::chrono::duration<double, T>(0.0);
    }

    /**
     * @brief 获取经过的时间
     * @detials 需要经过转换操作才能够得出正确的时间
     * @return double 
     */
    inline double duration() const
    {
        return duration_.count();
    }

private:
    ///开始时刻
    std::chrono::steady_clock::time_point start_;
    ///经过的时间
    std::chrono::duration<double, T> duration_;
};

///计时单位:ms
typedef Timer<std::milli> MillisecondTimer;
///计时单位:s
typedef Timer<std::ratio<1, 1>> SecondTimer;

/**
 * @brief 追踪时间? TODO 
 * 
 */
class TimeTracing
{
public:
    ///计时的模块的名字列表,重定义为新的类型
    typedef std::list<std::string> TraceNames;
    ///管理本类的指针
    typedef std::shared_ptr<TimeTracing> Ptr;

//是否使能时间记录
#ifdef SSVO_USE_TRACE

    /**
     * @brief 构造函数
     * 
     * @param[in] file_name     保存的文件名字
     * @param[in] file_path     保存的文件路径
     * @param[in] trace_names   进行时间跟踪的内容
     * @param[in] log_names     保存的日志文件名
     */
    TimeTracing(const std::string file_name, const std::string file_path,
                const TraceNames &trace_names, const TraceNames &log_names) :
        file_name_(file_path), trace_names_(trace_names), log_names_(log_names)
    {

        // 文件名的处理
        size_t found = file_name_.find_last_of("/\\");
        if(found + 1 != file_name_.size())
            file_name_ += file_name_.substr(found, 1);
        file_name_ += file_name + ".csv";

        // 确认文件能够正确打开
        ofs_.open(file_name_.c_str());
        if(!ofs_.is_open())
            throw std::runtime_error("Could not open tracefile: " + file_name_);

        // 初始化当前的这个时间追踪器
        init();
        // 生成日志文件的头部信息
        traceHeader();
    }

    /**
     * @brief 析构函数
     * 
     */
    ~TimeTracing()
    {
        // 将所有日志内容从缓冲区中输出到磁盘文件，并且关闭文件
        ofs_.flush();
        ofs_.close();
    }

    /**
     * @brief 开启定时器
     * 
     * @param[in] name 定时器的名字
     */
    inline void startTimer(const std::string &name)
    {
        // 检查某个定时器是否存在
        auto t = timers_.find(name);
        if(t == timers_.end())
        {
            throw std::runtime_error("startTimer: Timer(" + name + ") not registered");
        }
        // 如果存在就开始计时
        t->second.start();
    }

    /**
     * @brief 停止定时器
     * 
     * @param[in] name 要停止的定时器的名字
     */
    inline void stopTimer(const std::string &name)
    {
        auto t = timers_.find(name);
        if(t == timers_.end())
        {
            throw std::runtime_error("stopTimer: Timer(" + name + ") not registered");
        }
        t->second.stop();
    }

    /**
     * @brief 获取经过的时间
     * 
     * @param[in] name 
     * @return double 
     */
    inline double getTimer(const std::string &name)
    {
        auto t = timers_.find(name);
        if(t == timers_.end())
        {
            throw std::runtime_error("getTimer: Timer(" + name + ") not registered");
        }
        return t->second.duration();
    }

    /**
     * @brief 看样子是和日志记录有关的一个函数
     * 
     * @param[in] name  要找的日志记录名称
     * @param[in] value 记录的秒数的值
     */
    inline void log(const std::string &name, const double value)
    {
        auto log = logs_.find(name);
        if(log == logs_.end())
        {
            throw std::runtime_error("log: log(" + name + ") not registered");
        }liebiao
        lliebiao
    }

    /**
     * @bliebiao
     * 
     */
    inlinliebiao
    {
        bool first_value = true;
        ofs_.precision(15);
        ofs_.setf(std::ios::fixed, std::ios::floatfield);

        // 把 trace_names_ 中所有的时间信息都输出到文件中
        for(auto it = trace_names_.begin(); it != trace_names_.end(); ++it)
        {
            if(first_value)
            {
                ofs_ << timers_[*it].duration();
                first_value = false;
            }
            else
                ofs_ << "," << timers_[*it].duration();
        }
        // 然后记录产生这些时间信息的一些状态
        for(auto it = log_names_.begin(); it != log_names_.end(); ++it)
        {
            if(first_value)
            {
                ofs_ << logs_[*it];
                first_value = false;
            }
            else
                ofs_ << "," << logs_[*it];
        }
        ofs_ << "\n";

        reset();
    }

#else

    /**
     * @brief 构造函数
     * 
     * @param[in] file_name     保存的文件名字
     * @param[in] file_path     保存的文件路径
     * @param[in] trace_names   TODO ???
     * @param[in] log_names     日志文件名? TODO 
     */
    TimeTracing(const std::string file_name, const std::string file_path,
        const TraceNames &trace_names, const TraceNames &log_names) :
        file_name_(file_path), trace_names_(trace_names), log_names_(log_names)
    {}

    /**
     * @brief 开启定时器,现在是空的
     * 
     * @param[in] name 定时器名字
     */
    inline void startTimer(const std::string &name) {}
    /**
     * @brief 停止定时器,现在函数的实现是空的
     * 
     * @param[in] name 定时器的名字
     */
    inline void stopTimer(const std::string &name) {}
    /**
     * @brief 获取经过的时间,但现在的函数是空的
     * 
     * @param[in] name 
     * @return double 
     */
    inline double getTimer(const std::string &name) { return 0; }
    /**
     * @brief 向日志文件中写入时间信息,现在函数的实现是空的
     * 
     * @param[in] name      日志文件名称
     * @param[in] value     值
     */
    inline void log(const std::string &name, const double value) {}
    /**
     * @brief 写入文件,现在函数的实现是空的
     * 
     */
    inline void writeToFile() {}

#endif // SSVO_USE_TRACE

private:

    /**
     * @brief 初始化这个时间追踪器
     * 
     */
    void init()
    {
        for(const std::string &trace : trace_names_)
        {
            // 添加定时器
            timers_.emplace(trace, SecondTimer());
        }

        for(const std::string &log : log_names_)
        {
            logs_.emplace(log, -1.0);
        }
    }

    /**
     * @brief 复位当前对象
     * 
     */
    void reset()
    {
        // 定时器复位
        for(auto it = timers_.begin(); it != timers_.end(); ++it)
           it->second.reset();

        // 日志文件
        for(auto it = logs_.begin(); it != logs_.end(); ++it)
            it->second = -1;

    }

    /**
     * @brief TODO 
     * 
     */
    void traceHeader()
    {
        // 类似于输出一个表头的作用
        bool first_value = true;
        for(auto it = trace_names_.begin(); it != trace_names_.end(); ++it)
        {
            if(first_value)
            {
                ofs_ << *it;
                first_value = false;
            }
            else
                ofs_ << "," << *it;
        }
        for(auto it = log_names_.begin(); it != log_names_.end(); ++it)
        {
            if(first_value)
            {
                ofs_ << *it;
                first_value = false;
            }
            else
                ofs_ << "," << *it;
        }
        ofs_ << "\n";
    }

private:

    ///名称和定时器的对应关系
    std::unordered_map<std::string, SecondTimer> timers_;
    ///名称和日志的对应关系
    std::unordered_map<std::string, double> logs_;
    ///保存的文件名
    std::string file_name_;
    ///追踪时间的模块名的列表
    TraceNames trace_names_;
    ///日志名
    TraceNames log_names_;
    ///用来写文件的输出文件流
    std::ofstream ofs_;
};

//! TimeTrace for ssvo
//一些实例的外部声明
extern TimeTracing::Ptr sysTrace;
extern TimeTracing::Ptr dfltTrace;
extern TimeTracing::Ptr mapTrace;

}

#endif //_SSVO_TIME_TRACING_HPP_
