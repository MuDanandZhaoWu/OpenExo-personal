#ifndef TIME_HELPER_H
#define TIME_HELPER_H

#define MAX_TICKERS 15

#include <vector>

/* Class to help track the time of code execution. The class uses a singleton design pattern. 
 * 
 Example usage: 
 *      Time_Helper* my_time_helper_singleton = get_instance();
 *      static const float my_context = my_time_helper_singleton->generate_new_context();
 *      static my_delta_time;
 *      my_delta_time = my_time_helper_singleton->tick(my_context);
 *
 * The above code gets a reference to the singleton and uses it to generate a persisent context (see below). 
 * 'my_delta_time' will be filled with the time between the calls to tick in millis (first call will return 0).
 * 
 * When you are done with a context, clean it up using 'destroy_context'. If you would like to use microseconds,
 * the default value of 'use_micros' in the constructor should be true. 
 * 
 * If 'tick()' is continuosly returning 0, you are passing an invalid context. 
 *
 */

/* 
 * 用于辅助追踪代码执行耗时的类。该类采用单例设计模式。
 * 
 * 使用示例：
 *      Time_Helper* my_time_helper_singleton = get_instance();
 *      static const float my_context = my_time_helper_singleton->generate_new_context();
 *      static my_delta_time;
 *      my_delta_time = my_time_helper_singleton->tick(my_context);
 *
 * 上述代码获取单例实例，并利用其生成一个持久化上下文（详见下文）。
 * 'my_delta_time' 会被赋值为两次调用 tick 之间的时间间隔，单位为毫秒（首次调用返回 0）。
 * 
 * 不再使用某个上下文时，通过 'destroy_context' 对其进行清理。若需要使用微秒作为单位，
 * 构造函数中 'use_micros' 的默认值应设为 true。
 * 
 * 若 'tick()' 持续返回 0，说明传入的上下文无效。
 *
 */

typedef struct {
    float context;
    float old_time = -1;
    int k_index;
} ticker_t;

class Time_Helper
{
    public:
        Time_Helper(bool use_micros = true);
        static Time_Helper* get_instance();

        float peek(float context);
        float tick(float context);

        float generate_new_context();
        void destroy_context(float context);
    
    private:
        bool _context_conflicts(float context);
        ticker_t* _ticker_from_context(float context);

        int ticker_count = 0;
        std::vector<ticker_t> tickers;

        bool _k_use_micros;
};

#endif