#include "Time_Helper.h"
#include "Logger.h"
#include <Arduino.h>

/* Public */

/*
决定时间测量是使用微秒(micros())还是毫秒(millis())，默认为微秒
*/
Time_Helper::Time_Helper(bool use_micros)
{
    _k_use_micros = use_micros;
}

//创建单例实例
Time_Helper* Time_Helper::get_instance()
{
    static Time_Helper* instance = new Time_Helper;
    return instance;
}

/*
计算当前时间与指定上下文（context）的上一次记录时间之间的时间差，
但不会更新上下文的时间记录（即 old_time 字段）。它主要用于查看时间间隔，
而不影响计时器的状态。
*/
float Time_Helper::peek(float context)
{
    //判断计时单位
    float new_time = ((_k_use_micros) ? (micros()):(millis()));
    
    //遍历容器, 查找是否存在与传入的 context 值匹配的 ticker_t 结构体
    ticker_t* ticker = _ticker_from_context(context);
    
    // 上下文不存在，或这是该计时器的首次计时       The context does not exist or this is the tickers first tick
    //如果 k_index < 0，说明当前 ticker_t 是一个无效的对象（通常是 _ticker_from_context 返回的默认错误对象 err_ticker, err_ticker 中定义 old_time = -1, k_index = -1）
    //如果 old_time < 0，说明该计时器尚未记录任何时间（即这是该计时器的首次使用）
    if (ticker->k_index < 0 || ticker->old_time < 0) {
        return 0;
    }
    
    //返回当前时间与上一次记录时间之间的时间差
    return new_time - ticker->old_time;
}

/*
计算当前时间与指定上下文（context）的上一次记录时间之间
的时间差，并更新该上下文的时间记录（old_time 字段）
*/
float Time_Helper::tick(float context)
{
    float new_time;
    if (_k_use_micros) 
    {
        new_time = micros();
    }
    else
    {
        new_time = millis();
    }
    
    ticker_t* ticker = _ticker_from_context(context);
    
    //The context does not exist or this is the tickers first tick
    if (ticker->k_index < 0 || ticker->old_time < 0) {
        return 0;
    }
    
    //计算当前时间与上一次记录时间之间的时间差，并更新 old_time 字段为当前时间
    float return_time = new_time - ticker->old_time;
    ticker->old_time = new_time;
    return return_time;
}

float Time_Helper::generate_new_context()
{
    if (ticker_count == (MAX_TICKERS - 1)) {
        return 0;
    }

    //寻找不重合的context值, 以避免ticker_t实例之间的干扰
    bool searching = true;
    float found;
    while (searching) {
        float new_context = random(1000);
        //判断新生成的 new_context 值是否与现有的 ticker_t 实例的 context 值发生冲突（即是否已经存在相同的 context）。如果没有冲突，则将该值赋给 found，并结束搜索。
        if (!_context_conflicts(new_context)) {
            found = new_context;
            searching = false;
        }
    }

    //Track new ticker instance
    ticker_t* new_ticker = new ticker_t;
    new_ticker->context = found;
    //初始化新 ticker 的 old_time 字段为 0，表示该计时器尚未记录过时间（即这是该计时器的首次使用）。这与之前在 peek() 和 tick()
    new_ticker->old_time = 0;
    //将新 ticker 的 k_index 字段设置为当前的 ticker_count 值，以便在 tickers 容器中快速定位该 ticker 的位置。ticker_count 代表当前已存在的 ticker 数量，因此新 ticker 的索引将是当前数量的值。
    new_ticker->k_index = ticker_count;
    //将新创建的 ticker_t 实例添加到 tickers 容器中
    tickers.push_back(*new_ticker); 

    return found;
}

//根据传入的 context 值，查找对应的 ticker_t 实例，并将其从 tickers 容器中移除，以销毁该计时器实例
void Time_Helper::destroy_context(float context)
{
    ticker_t* ticker_to_destroy = _ticker_from_context(context);
    tickers.erase(tickers.begin()+(ticker_to_destroy->k_index-1));
    ticker_count--;
}

/* Private */
//检查新建的 new context 是否与已有的 context 值冲突
bool Time_Helper::_context_conflicts(float context)
{
    for (int i=0; i < tickers.size(); i++) {
        if (context == tickers[i].context) {
            return true;
        }
    }
    return false;
}

/**
 * 遍历tickers容器, 查找与传入的 context 值匹配的 ticker_t 对象
 * @param context 要查找的上下文值
 * @return 匹配的ticker_t对象指针，如果未找到则返回错误ticker
 */
ticker_t* Time_Helper::_ticker_from_context(float context)
{
    //对 err_ticker 的定义, 
    static ticker_t err_ticker = {
        .context = 0,
        .old_time = -1,
        .k_index = -1
    };

    //遍历容器过程, 如果未找到匹配的容器, 返回一个错误ticker
    for (int i=0; i < tickers.size(); i++) {
        if (context == tickers[i].context) {
            return &tickers[i];
        }
    }
    return &err_ticker;
}