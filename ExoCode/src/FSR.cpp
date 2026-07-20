
#include "FSR.h"

#include "Board.h"
#include "Logger.h"
//#define FSR_DEBUG 1   //Uncomment if you want to print debug statements

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
/*
 * Constructor for the force sensitive resistor
 * Takes in the pin to use and sets it as an analog input
 * Calibration and readings are initialized to 0 
 */
FSR::FSR(int pin)
{
    _pin = pin;
    
    _raw_reading = 0;
    _calibrated_reading = 0;
    
    _last_do_calibrate = false; 
    _start_time = 0;
    _calibration_min = 0;
    _calibration_max = 0;
    
    _state = false;
    _last_do_refinement = false;
    _step_count = 0;
    _calibration_refinement_min = 0;
    _calibration_refinement_max = 0;
    
    #ifdef FSR_DEBUG
        logger::println("FSR:: Constructor : Exit");
    #endif
}

bool FSR::calibrate(bool do_calibrate)
{
    //检测标定触发信号的上升沿，并启动定时器        Check for rising edge of do_calibrate and start the timer
    if (do_calibrate > _last_do_calibrate)
    {
        // logger::print("FSR::calibrate : Starting Cal for pin - ");
        // logger::println(_pin);
        
        _start_time = millis();

        /* 
           读取当前FSR的模拟值，并将其作为初始的最大值和最小值（刚开始的一瞬间只有一个采样点）
           Set the Max & Min Values 
        */
        _calibration_max = analogRead(_pin);
        _calibration_min = _calibration_max;
    }
    
    //记录当前校准已经进行了多久        Check if we are within the time window and need to do the calibration
    uint16_t delta = millis()-_start_time;
    // logger::print("FSR::calibrate : delta - ");
    // logger::println(delta);
    
    /*
    _cal_time在FSR.h中被定义为5000（也就是5秒)）
    do_calibrate是一个bool类型的标志位
    整句判断的意思是：如果当前的时间差小于等于5秒，并且do_calibrate为true，则继续进行标定
    */
    if((_cal_time >= (delta)) & do_calibrate)
    {
        // logger::print("FSR::calibrate : Continuing Cal for pin - ");
        // logger::println(_pin);

        uint16_t current_reading = analogRead(_pin);

        //得出最大最小阈值区间      Track the min and max.
        _calibration_max = max(_calibration_max, current_reading);
        _calibration_min = min(_calibration_min, current_reading);
    } 

    //The time window ran out so we are done.
    else if (do_calibrate)
    {
        // logger::print("FSR::calibrate : FSR Cal Done for pin - ");
        // logger::println(_pin);
        // logger::print("FSR::calibrate : _calibration_max - ");
        // logger::print(_calibration_max);
        // logger::print("\n");
        do_calibrate = false;
    }
        
    //Store the reading for next time.
    _last_do_calibrate = do_calibrate;
    
    return do_calibrate;
};

bool FSR::refine_calibration(bool do_refinement)
{
    if (do_refinement)
    {
        //Check for rising edge of do_calibrate
        if (do_refinement > _last_do_refinement)
        {
            //步数计数器
            _step_count = 0;
            
            /*
            将单步最大、最小值缓存初始化为标定区间中点，防止初始值干扰极值采集        Set the step max min to the middle value so the initial value is likely not used.
            
            用粗校准范围的中点初始化当前步的最大/最小值追踪器，避免 _step_min 或 _step_max 被不合理初值污染，让后续能正确统计每一步里的真实最大值和最小值
            */
            _step_max = (_calibration_max+_calibration_min)/2;      //这里取的是粗校准得出的最大最小值进行取中点
            _step_min = (_calibration_max+_calibration_min)/2;

            //压力阈值平均值容器        Reset the sum that will be used for averaging
            _step_max_sum = 0;
            _step_min_sum = 0;
        }
        
        //判断当前经历的步数是否达到设定值(当前_num_steps设定值为7)     Check if we are done with the calibration
        if (_step_count < _num_steps)
        {
            uint16_t current_reading = analogRead(_pin);

            //For each step find max and min for every step, keep a running record of the max and min for the step.
            _step_max = max(_step_max, current_reading);
            _step_min = min(_step_min, current_reading);
            
            //Store the current state so we can check for change
            bool last_state = _state;
            _state = utils::schmitt_trigger(current_reading, last_state, _lower_threshold_percent_calibration_refinement * (_calibration_max-_calibration_min) + _calibration_min, _upper_threshold_percent_calibration_refinement * (_calibration_max-_calibration_min) + _calibration_min); 
            
            //检测到信号由低电平跳转为高电平，代表当前标定步骤采集完成，将本步的最大值、最小值分别累加至各自的总和变量      There is a new low -> high transition (next step), add the step max and min to their respective sums.
            //当 FSR 状态从低变高时(即_state从低变高时) 程序认为发生了一次新的踩地/步态触发，然后把这一阶段统计到的 _step_max 和 _step_min 加入总和
            if (_state > last_state) 
            {
                _step_max_sum = _step_max_sum + _step_max;
                _step_min_sum = _step_min_sum + _step_min;
                
                //Reset the step max/min tracker for the next step
                _step_max = (_calibration_max+_calibration_min)/2;
                _step_min = (_calibration_max+_calibration_min)/2;
                
                _step_count++;
                
                // logger::print("FSR::refine_calibration : New Step - ");
            }

        }
        else //_step_count到达设定值时完成校准      We are still at do_refinement but the _step_count is at the _num_steps
        {
            //Set the calibration as the average of the max values; average max and min, offset by min and normalize by (max-min), (val-avg_min)/(avg_max-avg_min)
            /*
               将精细标定区间设置为多轮采样最大值、最小值的平均值；
               把 7 次步态采集到的每一步最大压力取平均，存入_calibration_refinement_max（精细标定上限）；
               把 7 次步态采集到的每一步最小压力取平均，存入_calibration_refinement_min（精细标定下限）；
            */
            _calibration_refinement_max = static_cast<decltype(_calibration_refinement_max)>(_step_max_sum)/_num_steps;     //Casting to the type of _calibration_refinement_max before division 
            _calibration_refinement_min = static_cast<decltype(_calibration_refinement_min)>(_step_min_sum)/_num_steps;     //Casting to the type of _calibration_refinement_min before division 
 
            //将精细化校准标记设为false，表示校准完成
            do_refinement = false;
        } 
    }

    //校准完成之后将_last_do_refinement也设定为false，方便下次上升沿的检测        Store the value so we can check for a rising edge next time.
    _last_do_refinement = do_refinement;
    
    return do_refinement;
};

float FSR::read()
{
    //读取原始采样值
    _raw_reading = analogRead(_pin);

    //由于_calibration_refinement_max被初始化为零, 这里实际是在验证是否有进行精细化标定     Return the value using the calibrated refinement if it is done.
    const float refinement_span =
        _calibration_refinement_max - _calibration_refinement_min;
    const float calibration_span = _calibration_max - _calibration_min;
    if (_calibration_refinement_max > 0 && isfinite(refinement_span) &&
        refinement_span > 0.000001f)
    {
        //进行了精细校准的归一化步骤
        _calibrated_reading = ((float)_raw_reading - _calibration_refinement_min)/(_calibration_refinement_max-_calibration_refinement_min);
    }

    //If we haven't refined yet just use the regular calibration.
    else if (_calibration_max > 0 && isfinite(calibration_span) &&
             calibration_span > 0.000001f)
    {        
    //只进行了粗校准的归一化步骤
        _calibrated_reading = ((float)_raw_reading - _calibration_min)/(_calibration_max-_calibration_min);
    }

    //未进行校准的情况,直接回读原始数据     If no calibrations are done just return the raw reading.
    else
    {
        _calibrated_reading = _raw_reading;
    }

    if (!isfinite(_calibrated_reading))
    {
        _calibrated_reading = 0.0f;
    }
    
    //Based on the readings update the ground contact state.
    _calc_ground_contact();
    
    return  _calibrated_reading;

};

bool FSR::_calc_ground_contact()
{
    //建立一个临时变量保存本次判断结果，默认认为没有接触地面    Only do this if the refinement is done.
    bool current_state_estimate = false;

    //只有在精细化标定完成之后才进行施密特触发器判断，否则直接返回false
    const float refinement_span =
        _calibration_refinement_max - _calibration_refinement_min;
    if (_calibration_refinement_max > 0 && isfinite(refinement_span) &&
        refinement_span > 0.000001f && isfinite(_calibrated_reading))
    {
        current_state_estimate = utils::schmitt_trigger(_calibrated_reading, _ground_contact, _lower_threshold_percent_ground_contact, _upper_threshold_percent_ground_contact);
    }

    _ground_contact = current_state_estimate;

    return _ground_contact;
};

bool FSR::get_ground_contact()
{
    #ifdef FSR_DEBUG
        logger::print("FSR::refine_calibration : FSR pin - ");
        logger::print(_pin);
        logger::print("\t _ground_contact -");
        logger::println(_ground_contact);
    #endif
    return _ground_contact;
};

void FSR::get_contact_thresholds(float &lower_threshold_percent_ground_contact, float &upper_threshold_percent_ground_contact)
{
    lower_threshold_percent_ground_contact = _lower_threshold_percent_ground_contact;
    upper_threshold_percent_ground_contact = _upper_threshold_percent_ground_contact;
};

void FSR::set_contact_thresholds(float lower_threshold_percent_ground_contact, float upper_threshold_percent_ground_contact)
{
    _lower_threshold_percent_ground_contact = lower_threshold_percent_ground_contact;
    _upper_threshold_percent_ground_contact = upper_threshold_percent_ground_contact;
};

FSR_Regressed::FSR_Regressed(int pin)
{
    _pin = pin;
    
    _raw_reading = 0;
    _calibrated_reading = 0;
    
    _last_do_calibrate = false; 
    _start_time = 0;
    _calibration_min = 0;
    _calibration_max = 0;
    
    _state = false;
    _last_do_refinement = false;
    _step_count = 0;
    _calibration_refinement_min = 0;
    _calibration_refinement_max = 0;
    
    #ifdef FSR_DEBUG
        logger::println("FSR:: Constructor : Exit");
    #endif
}

//此校准会对 ADC 原始值做多层转换 + 力矩回归拟合，所有校准全部基于换算后的力矩等效值
bool FSR_Regressed::calibrate(bool do_calibrate)
{
    // 检测标定触发信号的上升沿，并启动定时器       Check for rising edge of do_calibrate and start the timer
    if (do_calibrate > _last_do_calibrate)
    {
        // logger::print("FSR::calibrate : Starting Cal for pin - ");
        // logger::println(_pin);

        _start_time = millis();

        /* Set the Max & Min Values */
        /* Regression Equation FSR to Make Proportional to Ankle Moment*/
        /* 设置最大值与最小值 */
        /* FSR回归拟合方程，用于使传感输出与踝关节力矩成比例 */
        /*
        double p[4]数组保存的是一个三次多项式的固定系数
        这些系数来自预先完成的实验回归拟合，不是在当前校准过程中计算出来的
        */
        double p[4] = { 0.0787, -0.8471, 20.599, -22.670 };
        /*
        10 * 3.3 * analogRead(_pin) / 4095:
        3.3 V 参考电压换算,12位ADC采样值转换为电压值
        10* 是人为增加的比例系数,为了放大信号幅值, 放大输出信号幅值
        */
        float Vo = 10 * 3.3 * analogRead(_pin) / 4095;            
        /*
        使用Interlink FSR经验公式，将前面的电压相关量转换成与 FSR 受力相关的量
        可简单理解为：ADC原始值 → 电压相关量 → FSR受力相关量
        */
        Vo = (Vo) / (87.43 * pow((Vo), (-0.6721)) - 7.883);                 //Apply interlink conversion
        //由于 FSR 的电阻、受力与电压通常不是线性关系，所以需要使用幂函数进行转换
        Vo = p[0] * Vo * Vo * Vo + p[1] * Vo * Vo + p[2] * Vo + p[3];       //Apply amplification polynomial
        Vo = (Vo < 0.2) ? (0) : (Vo);                                       //If the value is less than 0.2, set it to zero
        _calibration_max = Vo;
        _calibration_min = _calibration_max;
    }

    //Check if we are within the time window and need to do the calibration
    uint16_t delta = millis() - _start_time;
    // logger::print("FSR::calibrate : delta - ");
    // logger::println(delta);

    //_cal_time初设为5000ms，也就是5秒，如果超过5秒，则认为已经完成标定
    if ((_cal_time >= (delta)) & do_calibrate)
    {
        // logger::print("FSR::calibrate : Continuing Cal for pin - ");
        // logger::println(_pin);

        /* Regression Equation FSR to Make Proportional to Ankle Moment*/
        double p[4] = { 0.0787, -0.8471, 20.599, -22.670 };
        float Vo = 10 * 3.3 * analogRead(_pin) / 4095;                      //ZL Added in the 10* to scale the output
        Vo = (Vo) / (87.43 * pow((Vo), (-0.6721)) - 7.883);                 //Apply interlink conversion
        Vo = p[0] * Vo * Vo * Vo + p[1] * Vo * Vo + p[2] * Vo + p[3];       //Apply amplification polynomial
        Vo = (Vo < 0.2) ? (0) : (Vo);                                       //If the value is less than 0.2, set it to zero
        float current_reading = Vo;

        //Track the min and max.
        _calibration_max = max(_calibration_max, current_reading);
        _calibration_min = min(_calibration_min, current_reading);
    }

    //The time window ran out so we are done.
    else if (do_calibrate)
    {
        // logger::print("FSR::calibrate : FSR Cal Done for pin - ");
        // logger::println(_pin);
        // logger::print("FSR::calibrate : _calibration_max - ");
        // logger::print(_calibration_max);
        // logger::print("\n");
        do_calibrate = false;
    }

    //Store the reading for next time.
    _last_do_calibrate = do_calibrate;

    return do_calibrate;
};

bool FSR_Regressed::refine_calibration(bool do_refinement)
{
    if (do_refinement)
    {
        //Check for rising edge of do_calibrate
        if (do_refinement > _last_do_refinement)
        {
            _step_count = 0;

            //Set the step max min to the middle value so the initial value is likely not used.
            _step_max = (_calibration_max + _calibration_min) / 2;
            _step_min = (_calibration_max + _calibration_min) / 2;

            //Reset the sum that will be used for averaging
            _step_max_sum = 0;
            _step_min_sum = 0;
        }

        //Check if we are done with the calibration
        if (_step_count < _num_steps)
        {
            /* Regression Equation FSR to Make Proportional to Ankle Moment*/
            double p[4] = { 0.0787, -0.8471, 20.599, -22.670 };
            float Vo = 10 * 3.3 * analogRead(_pin) / 4095;                      //ZL Added in the 10* to scale the output
            Vo = (Vo) / (87.43 * pow((Vo), (-0.6721)) - 7.883);                 //Apply interlink conversion
            Vo = p[0] * Vo * Vo * Vo + p[1] * Vo * Vo + p[2] * Vo + p[3];       //Apply amplification polynomial
            Vo = (Vo < 0.2) ? (0) : (Vo);                                       //If the value is less than 0.2, set it to zero
            float current_reading = Vo;

            //For each step find max and min for every step, keep a running record of the max and min for the step.
            _step_max = max(_step_max, current_reading);
            _step_min = min(_step_min, current_reading);

            //Store the current state so we can check for change
            bool last_state = _state;
            _state = utils::schmitt_trigger(current_reading, last_state, _lower_threshold_percent_calibration_refinement * (_calibration_max - _calibration_min) + _calibration_min, _upper_threshold_percent_calibration_refinement * (_calibration_max - _calibration_min) + _calibration_min);

            //There is a new low -> high transition (next step), add the step max and min to their respective sums.
            if (_state > last_state)
            {
                _step_max_sum = _step_max_sum + _step_max;
                _step_min_sum = _step_min_sum + _step_min;

                //Reset the step max/min tracker for the next step
                _step_max = (_calibration_max + _calibration_min) / 2;
                _step_min = (_calibration_max + _calibration_min) / 2;

                _step_count++;

                // logger::print("FSR::refine_calibration : New Step - ");
            }

        }
        else //We are still at do_refinement but the _step_count is at the _num_steps
        {
            //Set the calibration as the average of the max values; average max and min, offset by min and normalize by (max-min), (val-avg_min)/(avg_max-avg_min)
            _calibration_refinement_max = static_cast<decltype(_calibration_refinement_max)>(_step_max_sum) / _num_steps;     //Casting to the type of _calibration_refinement_max before division 
            _calibration_refinement_min = static_cast<decltype(_calibration_refinement_min)>(_step_min_sum) / _num_steps;     //Casting to the type of _calibration_refinement_max before division 

            //Refinement is done
            do_refinement = false;
        }
    }

    //Store the value so we can check for a rising edge next time.
    _last_do_refinement = do_refinement;

    return do_refinement;
};


float FSR_Regressed::read()
{
    /* Regression Equation FSR to Make Proportional to Ankle Moment*/
    double p[4] = { 0.0787, -0.8471, 20.599, -22.670 };
    float Vo = 10 * 3.3 * analogRead(_pin) / 4095;                      //ZL Added in the 10* to scale the output
    Vo = (Vo) / (87.43 * pow((Vo), (-0.6721)) - 7.883);                 //Apply interlink conversion
    Vo = p[0] * Vo * Vo * Vo + p[1] * Vo * Vo + p[2] * Vo + p[3];       //Apply amplification polynomial
    Vo = (Vo < 0.2) ? (0) : (Vo);                                       //If the value is less than 0.2, set it to zero
    float _raw_reading = Vo;

    //Return the value using the calibrated refinement if it is done.
    const float refinement_span =
        _calibration_refinement_max - _calibration_refinement_min;
    const float calibration_span = _calibration_max - _calibration_min;
    if (_calibration_refinement_max > 0 && isfinite(refinement_span) &&
        refinement_span > 0.000001f)
    {
        _calibrated_reading = ((float)_raw_reading - _calibration_refinement_min) / (_calibration_refinement_max - _calibration_refinement_min);
    }

    //If we haven't refined yet just use the regular calibration.
    else if (_calibration_max > 0 && isfinite(calibration_span) &&
             calibration_span > 0.000001f)
    {
        _calibrated_reading = ((float)_raw_reading - _calibration_min) / (_calibration_max - _calibration_min);
    }

    //If no calibrations are done just return the raw reading.
    else
    {
        _calibrated_reading = _raw_reading;
    }


    if (!isfinite(_calibrated_reading))
    {
        _calibrated_reading = 0.0f;
    }

    //Based on the readings update the ground contact state.
    _calc_ground_contact();

    return  _calibrated_reading;
};

bool FSR_Regressed::_calc_ground_contact()
{
    //Only do this if the refinement is done.
    bool current_state_estimate = false;

    const float refinement_span =
        _calibration_refinement_max - _calibration_refinement_min;
    if (_calibration_refinement_max > 0 && isfinite(refinement_span) &&
        refinement_span > 0.000001f && isfinite(_calibrated_reading))
    {
        current_state_estimate = utils::schmitt_trigger(_calibrated_reading, _ground_contact, _lower_threshold_percent_ground_contact, _upper_threshold_percent_ground_contact);
    }

    _ground_contact = current_state_estimate;

    return _ground_contact;
};


bool FSR_Regressed::get_ground_contact()
{
    // logger::print("FSR::refine_calibration : FSR pin - ");
    // logger::print(_pin);
    // logger::print("\t _ground_contact -");
    // logger::println(_ground_contact);
    return _ground_contact;
};

void FSR_Regressed::get_contact_thresholds(float &lower_threshold_percent_ground_contact, float &upper_threshold_percent_ground_contact)
{
    lower_threshold_percent_ground_contact = _lower_threshold_percent_ground_contact;
    upper_threshold_percent_ground_contact = _upper_threshold_percent_ground_contact;
};

void FSR_Regressed::set_contact_thresholds(float lower_threshold_percent_ground_contact, float upper_threshold_percent_ground_contact)
{
    _lower_threshold_percent_ground_contact = lower_threshold_percent_ground_contact;
    _upper_threshold_percent_ground_contact = upper_threshold_percent_ground_contact;
};
#endif
