/*
 * P. Stegall Jan. 2022
*/

#include "Arduino.h" 

#include "Motor.h"
#include "CAN.h"
#include "ErrorManager.h"
#include "error_codes.h"
#include "Logger.h"
#include "ErrorReporter.h"
#include "error_codes.h"
//#define MOTOR_DEBUG           //Uncomment if you want to print debug statments to the serial monitor

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 


_Motor::_Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin)
{
    _id = id;
    //is_left的计算路径
    _is_left = ((uint8_t)this->_id & (uint8_t)config_defs::joint_id::left) == (uint8_t)config_defs::joint_id::left;
    _data = exo_data;
    _enable_pin = enable_pin;
    _prev_motor_enabled = false; 
    _prev_on_state = false;
    
    #ifdef MOTOR_DEBUG
        logger::print("_Motor::_Motor : _enable_pin = ");
        logger::print(_enable_pin);
        logger::print("\n");
    #endif
    
    pinMode(_enable_pin, OUTPUT);
    
    //将_motor_data指向所应用关节的电机数据     Set _motor_data to point to the data specific to this motor.
    switch (utils::get_joint_type(_id))
    {
        case (uint8_t)config_defs::joint_id::hip:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.hip.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.hip.motor);
            }
            break;
            
        case (uint8_t)config_defs::joint_id::knee:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.knee.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.knee.motor);
            }
            break;
        
        case (uint8_t)config_defs::joint_id::ankle:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.ankle.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.ankle.motor);
            }
            break;
        case (uint8_t)config_defs::joint_id::elbow:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.elbow.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.elbow.motor);
            }
            break;
        case (uint8_t)config_defs::joint_id::arm_1:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.arm_1.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.arm_1.motor);
            }
            break;
        case (uint8_t)config_defs::joint_id::arm_2:
            if (_is_left)
            {
                _motor_data = &(exo_data->left_side.arm_2.motor);
            }
            else
            {
                _motor_data = &(exo_data->right_side.arm_2.motor);
            }
            break;
    }

    #ifdef MOTOR_DEBUG
        logger::println("_Motor::_Motor : Leaving Constructor");
    #endif

};

bool _Motor::get_is_left() 
{
    return _is_left;
};

config_defs::joint_id _Motor::get_id()
{
    return _id;
};

/*
 * Constructor for the CAN Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the CAN (e.g. TMotor) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model (e.g. AK60), which may have specific torque constants etc.
 */
_CANMotor::_CANMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin) //Constructor: type is the motor type
: _Motor(id, exo_data, enable_pin)
{
    //重定义电机参数的上下限值，对齐官方给出的电机数据
    _KP_MIN = 0.0f;
    _KP_MAX = 500.0f;
    _KD_MIN = 0.0f;
    _KD_MAX = 5.0f;
    _P_MAX = 12.5f;

    JointData* j_data = exo_data->get_joint_with(static_cast<uint8_t>(id));
    j_data->motor.kt = this->get_Kt();

    _enable_response = false;
    _disable_pending = false;
    _disable_zero_queued = false;
    _power_restore_pending = true;

    #ifdef MOTOR_DEBUG
        logger::println("_CANMotor::_CANMotor : Leaving Constructor");
    #endif
};


// 一次完整 CAN 通讯事务，是所有 AK 系列 CAN 总线电机（AK60/AK60v3/AK70/AK80 等）单控制周期的标准交互入口
void _CANMotor::transaction(float torque)
{
    //Send data and read response 
    send_data(torque);      // 1.下发控制指令
    read_data();            // 2.读取电机反馈报文
    check_response();       // 3.校验通讯链路健康状态
};

//读取电机发出的数据
void _CANMotor::read_data()
{
    if (_motor_data->enabled)
    {
        CAN* can = can->getInstance();
        int direction_modifier = _motor_data->flip_direction ? -1 : 1;

        // Determine if the motor type is AK60v3 (extended, new format) or old AK (standard, old format)
		// Background: AK60v3 motors employ a different communication protocol and are handled differently in this context.
        bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

        CAN_message_t msg;
        uint32_t received_us = 0;
        if (!can->read_for_motor((uint8_t)_motor_data->id, is_ak60v3, msg, received_us))
        {
            _handle_read_failure();
            return;
        }
        bool valid_response = false;
    
        // AK60v3电机对应的逻辑     When the motor type is AK60v3        
        if (is_ak60v3) {
            if (msg.len < 6 || !msg.flags.extended) {
                _handle_read_failure();
                return;
            }
            // AK60v3: NEW message format
            if ((msg.id & 0xFF) == uint8_t(_motor_data->id))
            {
                uint32_t p_int = (msg.buf[0] << 8) | msg.buf[1];        //电机位置
                uint32_t v_int = (msg.buf[2] << 8) | msg.buf[3];        //电机速度
                uint32_t i_int = (msg.buf[4] << 8) | msg.buf[5];        //电机电流
                _motor_data->p = direction_modifier * _uint_to_float(p_int, -_P_MAX, _P_MAX, 16);   
                _motor_data->v = direction_modifier * _uint_to_float(v_int, -_V_MAX, _V_MAX, 12);
                _motor_data->i = direction_modifier * _uint_to_float(i_int, -_I_MAX, _I_MAX, 12);
                #ifdef MOTOR_DEBUG
                    logger::print("_CANMotor::read_data():Got data-");
                    logger::print("ID:" + String(uint32_t(_motor_data->id)) + ",");
                    logger::print("P:"+String(_motor_data->p) + ",V:" + String(_motor_data->v) + ",I:" + String(_motor_data->i));
                    logger::print("\n");
                #endif
                _motor_data->timeout_count = 0;
                _motor_data->feedback_valid = true;
                _motor_data->last_feedback_us = received_us;
                ++_motor_data->feedback_sequence;
                _enable_response = true;
                valid_response = true;
            }
        }

		//AK60-6V1.1 KV80电机使用的逻辑     When the motor type is a CAN motor other than the AK60v3.
		else {
            if (msg.len < 6 || msg.flags.extended) {
                _handle_read_failure();
                return;
            }
            // 旧AK电机的0-7位数据对应的是电机驱动器ID, 就是我们在上位机设置的电机ID    Old AK: OLD message format
            if (msg.buf[0] == uint32_t(_motor_data->id))
            {
                uint32_t p_int = (msg.buf[1] << 8) | msg.buf[2];
                uint32_t v_int = (msg.buf[3] << 4) | (msg.buf[4] >> 4);
                uint32_t i_int = ((msg.buf[4] & 0xF) << 8) | msg.buf[5];
                _motor_data->p = direction_modifier * _uint_to_float(p_int, -_P_MAX, _P_MAX, 16);
                _motor_data->v = direction_modifier * _uint_to_float(v_int, -_V_MAX, _V_MAX, 12);
                _motor_data->i = direction_modifier * _uint_to_float(i_int, -_I_MAX, _I_MAX, 12);
                #ifdef MOTOR_DEBUG
                    logger::print("_CANMotor::read_data():Got data-");
                    logger::print("ID:" + String(uint32_t(_motor_data->id)) + ",");
                    logger::print("P:"+String(_motor_data->p) + ",V:" + String(_motor_data->v) + ",I:" + String(_motor_data->i));
                    logger::print("\n");
                #endif
                _motor_data->timeout_count = 0;
                _motor_data->feedback_valid = true;
                _motor_data->last_feedback_us = received_us;
                ++_motor_data->feedback_sequence;
                _enable_response = true;
                valid_response = true;
            }
        }
        if (!valid_response)
        {
            _handle_read_failure();
        }
    }
    return;
};

//向电机发数据, 传入电机扭矩参数
void _CANMotor::send_data(float torque)
{
    #ifdef MOTOR_DEBUG
        logger::print("Sending data: ");
        logger::print(uint32_t(_motor_data->id));
        logger::print("\n");
    #endif

    int direction_modifier = _motor_data->flip_direction ? -1 : 1;

    // A stop or motor error bypasses every ramp and forces the packed command
    // to zero. Keep sending the zero/disable transition so the drive cannot
    // retain the previous non-zero command.
    if (_data->estop || _error || !isfinite(torque))
    {
        torque = 0.0f;
        _motor_data->p_des = 0.0f;
        _motor_data->v_des = 0.0f;
        _motor_data->kp = 0.0f;
        _motor_data->kd = 0.0f;
    }

    //记录本次准备发送给电机的目标前馈扭矩
    _motor_data->t_ff = torque;
    //根据电机扭矩常数，把扭矩换算成电流
    const float kt = get_Kt();
    float current = 0.0f;
    if (isfinite(kt) && fabsf(kt) > 0.000001f)
    {
        current = torque / kt;
    }
    else
    {
        torque = 0.0f;
        _motor_data->t_ff = 0.0f;
    }
    if (!isfinite(current))
    {
        current = 0.0f;
        torque = 0.0f;
        _motor_data->t_ff = 0.0f;
    }

    /*
     *constrain函数, 数值钳位 / 限幅饱和，把输入值强制约束在 [最小值, 最大值] 区间内
     *如果 x < minVal → 返回 minVal（下限饱和）
     *如果 x > maxVal → 返回 maxVal（上限饱和）
     *如果 minVal ≤ x ≤ maxVal → 原样返回 x
     *最终发出的其实是经过各种嵌位、限幅等处理的p_des, v_des, kp, kd, i，后面的p_int等只是转换当中的中间变量
    */
    float p_sat = constrain(direction_modifier * _motor_data->p_des, -_P_MAX, _P_MAX);
    float v_sat = constrain(direction_modifier * _motor_data->v_des, -_V_MAX, _V_MAX);
    float kp_sat = constrain(_motor_data->kp, _KP_MIN, _KP_MAX);
    float kd_sat = constrain(_motor_data->kd, _KD_MIN, _KD_MAX);
    float i_sat = constrain(direction_modifier * current, -_I_MAX, _I_MAX);
    _motor_data->last_command = i_sat;
    uint32_t p_int = _float_to_uint(p_sat, -_P_MAX, _P_MAX, 16);
    uint32_t v_int = _float_to_uint(v_sat, -_V_MAX, _V_MAX, 12);
    uint32_t kp_int = _float_to_uint(kp_sat, _KP_MIN, _KP_MAX, 12);
    uint32_t kd_int = _float_to_uint(kd_sat, _KD_MIN, _KD_MAX, 12);
    uint32_t i_int = _float_to_uint(i_sat, -_I_MAX, _I_MAX, 12);

    // 创建CAN消息结构体 
    CAN_message_t msg;
    msg.len = 8;
    
    // Determine if this is an AK60v3 (extended, new format) or old AK (standard, old format)
    bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

    if (is_ak60v3) {
        // AK60v3: Extended CAN, NEW format
        msg.flags.extended = 1;
        msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;
        // NEW format
        msg.buf[0] = kp_int >> 4;
        msg.buf[1] = ((kp_int&0xF) << 4) | (kd_int >> 8);
        msg.buf[2] = (kd_int & 0xFF);
        msg.buf[3] = p_int >> 8;
        msg.buf[4] = p_int & 0xFF;
        msg.buf[5] = v_int >> 4;
        msg.buf[6] = ((v_int & 0xF) << 4) | (i_int >> 8);
        msg.buf[7] = i_int & 0xFF;
    } else {
        // 我们所使用的电机遵循的数据格式       Old AK: Standard CAN, OLD format
        msg.flags.extended = 0;
        msg.id = (uint32_t)_motor_data->id;
        // OLD format
        msg.buf[0] = p_int >> 8;
        msg.buf[1] = p_int & 0xFF;
        msg.buf[2] = v_int >> 4;
        msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
        msg.buf[4] = kp_int & 0xFF;
        msg.buf[5] = kd_int >> 4;
        msg.buf[6] = ((kd_int & 0xF) << 4) | (i_int >> 8);
        msg.buf[7] = i_int & 0xFF;
    }
    #ifdef MOTOR_DEBUG
        logger::print("_CANMotor::send_data::t_sat:: ");
        logger::print(torque);
        logger::print("\n");
    #endif

    CAN* can = can->getInstance();

    const bool drive_accepts_commands = is_ak60v3
        ? (_motor_data->enabled && !_error && !_data->estop)
        : _prev_motor_enabled;
    if (drive_accepts_commands)
    {
        //Set data in motor
        can->send(msg);
        _prev_motor_enabled = true;
    }

    /*
    如果当前 _motor_data->enabled == false，（也就是上一个if中_motor_data->enabled = false的的情况）
    但是上一轮 _prev_motor_enabled == true，
    说明电机刚刚从 enabled 变成 disabled, 
    这个时候代码会额外发送一次“零扭矩命令”。
    */
    else if (_prev_motor_enabled)
    {
        // AK60v3 skips enable(), so retry its final zero across cycles until
        // the frame has actually entered the local CAN TX queue.
        if (_queue_zero_command())
        {
            _prev_motor_enabled = false;
        }
    }
    return;
};

bool _CANMotor::_queue_zero_command()
{
    const uint32_t zero_p_int = _float_to_uint(0, -_P_MAX, _P_MAX, 16);
    const uint32_t zero_v_int = _float_to_uint(0, -_V_MAX, _V_MAX, 12);
    const uint32_t zero_kp_int = _float_to_uint(0, _KP_MIN, _KP_MAX, 12);
    const uint32_t zero_kd_int = _float_to_uint(0, _KD_MIN, _KD_MAX, 12);
    const uint32_t zero_i_int = _float_to_uint(0, -_I_MAX, _I_MAX, 12);

    CAN_message_t msg;
    msg.len = 8;
    const bool is_ak60v3 =
        _motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3;
    if (is_ak60v3)
    {
        msg.flags.extended = 1;
        msg.id = ((uint32_t)8 << 8) | (uint32_t)_motor_data->id;
        msg.buf[0] = zero_kp_int >> 4;
        msg.buf[1] = ((zero_kp_int & 0xF) << 4) | (zero_kd_int >> 8);
        msg.buf[2] = zero_kd_int & 0xFF;
        msg.buf[3] = zero_p_int >> 8;
        msg.buf[4] = zero_p_int & 0xFF;
        msg.buf[5] = zero_v_int >> 4;
        msg.buf[6] = ((zero_v_int & 0xF) << 4) | (zero_i_int >> 8);
        msg.buf[7] = zero_i_int & 0xFF;
    }
    else
    {
        msg.flags.extended = 0;
        msg.id = (uint32_t)_motor_data->id;
        msg.buf[0] = zero_p_int >> 8;
        msg.buf[1] = zero_p_int & 0xFF;
        msg.buf[2] = zero_v_int >> 4;
        msg.buf[3] = ((zero_v_int & 0xF) << 4) | (zero_kp_int >> 8);
        msg.buf[4] = zero_kp_int & 0xFF;
        msg.buf[5] = zero_kd_int >> 4;
        msg.buf[6] = ((zero_kd_int & 0xF) << 4) | (zero_i_int >> 8);
        msg.buf[7] = zero_i_int & 0xFF;
    }

    CAN* can = CAN::getInstance();
    return can->send(msg);
}

//基于最近电流反馈稳定程度的电机自动重使能检查
void _CANMotor::check_response()
{
    // Communication health is represented by feedback_valid/timestamp/sequence.
    // Never infer recovery from low current variance and never re-enable a
    // motor automatically after a controller or safety fault disabled it.
    if (!_motor_data->enabled || !_motor_data->feedback_valid)
    {
        return;
    }
    //仅当电机预设为使能状态时才运行本段逻辑        Only run if the motor is supposed to be enabled
    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) ||
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);

    if (_data->user_paused || !active_trial || _data->estop || _error)
    {
        return;
    }

    //向队列装填当前电流值      Measured current variance should be non-zero
    _measured_current.push(_motor_data->i);

    //队列大小判断, 当前队列大小设定为25个电流值
    if (_measured_current.size() > _current_queue_size)
    {
        //移除头部元素，保证队列大小不超过设定值      Pop the oldest value off the queue to maintain size
        _measured_current.pop();
        //方差计算
        auto pop_vals = utils::online_std_dev(_measured_current);

        // 仅当电机当前处于失能状态时，才尝试重新使能电机 (计算出的方差 < _variance_threshold：判定电流几乎无波动，CAN 通信失联，触发电机自动重使能自愈)
        // 恒扭矩运行时电流方差偏低属于正常情况，不应触发自动重使能逻辑
        (void)pop_vals;

    }
};

void _CANMotor::on_off()
{
    if (_data->estop || _error)
    {
        _motor_data->is_on = false;

        // logger::print("_CANMotor::on_off(bool is_on) : E-stop pulled - ");
        // logger::print(uint32_t(_motor_data->id));
        // logger::print("\n");
    }
    else if (_motor_data->enabled && !_motor_data->is_on)
    {
        // After a latched safety reset, power is restored only by a separate
        // explicit motor-enable request. enable() will condition the drive
        // with zero+disable before it permits an enable command.
        _motor_data->is_on = true;
    }

    if (_prev_on_state != _motor_data->is_on) //If was here to save time, can be removed if making problems, or add overide
    {
        if (_motor_data->is_on)
        {
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_on_state);

            #ifdef HEADLESS
                // Delay only on a real power-on edge, never every control loop.
                delay(2000);
            #endif

            // logger::print("_CANMotor::on_off(bool is_on) : Power on- ");
            // logger::print(uint32_t(_motor_data->id));
            // logger::print("\n");
        }
        else 
        {
            digitalWrite(_enable_pin, logic_micro_pins::motor_enable_off_state);
            _power_restore_pending = true;

            // logger::print("_CANMotor::on_off(bool is_on) : Power off- ");
            // logger::print(uint32_t(_motor_data->id));
            // logger::print("\n");
        }
    }
    _prev_on_state = _motor_data->is_on;

};

//函数重载, 是 bool _CANMotor::enable(bool overide) 函数的上层接口, 将电机驱动器的 CAN 使能状态同步为 _motor_data->enabled 指定的目标状态
bool _CANMotor::enable()
{
    return enable(false);
};

// 形参overide为强制覆盖开关
bool _CANMotor::enable(bool overide)
{
    #ifdef MOTOR_DEBUG
        //  logger::print(_prev_motor_enabled);
        //  logger::print("\t");
        //  logger::print(_motor_data->enabled);
        //  logger::print("\t");
        //  logger::print(_motor_data->is_on);
        //  logger::print("\n");
    #endif

    // 电机正确使能的逻辑
    const bool should_enable = _motor_data->enabled && !_error && !_data->estop;

    const bool needs_power_conditioning =
        should_enable && _power_restore_pending;
    
    //目标要关电机，上一轮电机是开启状态，且当前没有正在处理的停机流程
    if (!should_enable && _prev_motor_enabled && !_disable_pending)
    {
        _disable_pending = true;
        _disable_zero_queued = false;
    }
    //目标要开电机，但电机刚上电恢复（_power_restore_pending=true），硬件上电后驱动器状态未知，必须先完整归零失能，再允许使能
    if (needs_power_conditioning && !_disable_pending)
    {
        // After restoring hardware power, establish an all-zero/disabled
        // drive state first. The actual enable request is deferred one cycle.
        _disable_pending = true;
        _disable_zero_queued = false;
    }

    if (_disable_pending)
    {
        _motor_data->feedback_valid = false;
        _motor_data->p_des = 0.0f;
        _motor_data->v_des = 0.0f;
        _motor_data->kp = 0.0f;
        _motor_data->kd = 0.0f;

        if (!_disable_zero_queued)
        {
            _disable_zero_queued = _queue_zero_command();
        }

        const bool is_ak60v3 =
            _motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3;
        CAN_message_t disable_msg;
        disable_msg.len = 8;
        disable_msg.flags.extended = is_ak60v3 ? 1 : 0;
        disable_msg.id = is_ak60v3
            ? (((uint32_t)8 << 8) | (uint32_t)_motor_data->id)
            : (uint32_t)_motor_data->id;
        for (uint8_t i = 0; i < 7; ++i)
        {
            disable_msg.buf[i] = 0xFF;
        }
        disable_msg.buf[7] = 0xFD;

        // Preserve ordering even when the shared CAN TX queue is full: the
        // all-zero MIT command must enter the queue before the mode-disable
        // frame. If zero could not be queued this cycle, keep the whole stop
        // transition pending and try again on the next control cycle.
        CAN* can = CAN::getInstance();
        bool disable_queued = false;
        if (_disable_zero_queued)
        {
            disable_queued = can->send(disable_msg);
            for (uint8_t retry = 0; retry < 2 && !disable_queued; ++retry)
            {
                delayMicroseconds(50);
                disable_queued = can->send(disable_msg);
            }
        }

        const bool stop_queued = _disable_zero_queued && disable_queued;
        if (stop_queued)
        {
            // 停机程序执行结束后，清除标志位，允许下一轮使能请求
            _disable_pending = false;
            _disable_zero_queued = false;
            if (should_enable)
            {
                _power_restore_pending = false;
            }
        }

        _prev_motor_enabled = false;
        _enable_response = !should_enable && stop_queued;
        return should_enable ? false : stop_queued;
    }

    // Retry only an enable request that has not received a fresh response.
    // A disable request is a one-shot transition; repeatedly sending it would
    // block every control cycle and fill the shared RX FIFO with acknowledgments.
    /*
    _prev_motor_enabled != should_enable：电机目标状态和上一轮实际状态发生切换，必须更新指令；
    overide：外部传入 true，强制绕过节流规则，无视历史状态强制重发；
    should_enable && !_enable_response：目标要使能电机，但上一轮使能指令未收到电机应答（CAN 丢包 / 通讯超时），自动重试使能。
    */
    if ((_prev_motor_enabled != should_enable) || overide ||
        (should_enable && !_enable_response))
    {
        CAN_message_t msg;

        // Determine if this is an AK60v3 (extended format) or old AK (standard format)
        bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

        // Initialize message format fields
        msg.len = 8;
        if (is_ak60v3) {
            msg.flags.extended = 1;
            msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;
        } else {        //我们所使用的电机遵循的逻辑
            msg.flags.extended = 0;
            msg.id = (uint32_t)_motor_data->id;
        }

        msg.buf[0] = 0xFF;
        msg.buf[1] = 0xFF;
        msg.buf[2] = 0xFF;
        msg.buf[3] = 0xFF;
        msg.buf[4] = 0xFF;
        msg.buf[5] = 0xFF;
        msg.buf[6] = 0xFF;

        if (should_enable)
        {   //使能
            msg.buf[7] = 0xFC;
        }
        else
        {   //失能
            _motor_data->feedback_valid = false;
            msg.buf[7] = 0xFD;
        }

        CAN* can = can->getInstance();
        if (should_enable)
        {
            // Do not let a queued pre-enable feedback frame acknowledge this
            // enable request or become the controller's new neutral sample.
            // 不能用缓存队列里、本次使能操作之前的旧电机反馈帧，
            // 来判定本次使能指令应答成功，也不能把该旧帧数据当作控制器全新的零点基准采样值。
            //在本次使能前，清空“目标电机的历史接收报文”，保证随后读取到的反馈尽可能是本次使能之后产生的新反馈
            can->discard_for_motor((uint8_t)_motor_data->id, is_ak60v3);
        }

        bool command_queued = can->send(msg);
        if (!should_enable)
        {
            // Retry only failures to enter the local CAN TX queue; do not
            // block or continuously resend while waiting for a drive ACK.
            for (uint8_t retry = 0; retry < 2 && !command_queued; ++retry)
            {
                delayMicroseconds(50);
                command_queued = can->send(msg);
            }
        }
        if (should_enable && command_queued)
        {
            const uint32_t feedback_sequence_before = _motor_data->feedback_sequence;
            delayMicroseconds(500);
            read_data();
            _enable_response =
                _motor_data->feedback_sequence != feedback_sequence_before;
        }
        else if (should_enable)
        {
            // A late feedback frame must not acknowledge an enable command
            // that never entered the local TX queue. Keep both the command
            // and response state false so the next control cycle retries.
            _enable_response = false;
        }
        else
        {
            // Zero command + disable transition has been attempted, and a
            // safety fault additionally removes hardware enable power. Do not
            // repeatedly block the control loop waiting for an ACK.
            _enable_response = command_queued;
        }
        _prev_motor_enabled = should_enable && command_queued;
    }

    return should_enable ? _enable_response : true;
};

void _CANMotor::zero()
{
    CAN_message_t msg;

    // Determine if this is an AK60v3 (extended format) or old AK (standard format)
    bool is_ak60v3 = (_motor_data->motor_type == (uint8_t)config_defs::motor::AK60v3);

    // Initialize message format fields
    msg.len = 8;
    if (is_ak60v3) {
        msg.flags.extended = 1;
        msg.id = ((uint32_t) 8 << 8) | (uint32_t)_motor_data->id;
    } else {
        msg.flags.extended = 0;
        msg.id = uint32_t(_motor_data->id);
    }

    msg.buf[0] = 0xFF;
    msg.buf[1] = 0xFF;
    msg.buf[2] = 0xFF;
    msg.buf[3] = 0xFF;
    msg.buf[4] = 0xFF;
    msg.buf[5] = 0xFF;
    msg.buf[6] = 0xFF;
    msg.buf[7] = 0xFE;
    CAN* can = can->getInstance();
    can->send(msg);

    read_data();
};

float _CANMotor::get_Kt()
{
    return _Kt;     //获取电机扭矩常数
};

void _CANMotor::set_error()
{
    _error = true;
};

//设置电机扭矩常数
void _CANMotor::set_Kt(float Kt)
{
    _Kt = Kt;
};

void _CANMotor::_handle_read_failure()
{
    if (_motor_data->timeout_count < _motor_data->timeout_count_max)
    {
        ++_motor_data->timeout_count;
    }
    if (_motor_data->timeout_count >= _motor_data->timeout_count_max)
    {
        _motor_data->feedback_valid = false;
    }
};

float _CANMotor::_float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if (bits == 12) {
      pgg = (unsigned int) ((x-offset)*4095.0/span); 
    }
    if (bits == 16) {
      pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
};

float _CANMotor::_uint_to_float(unsigned int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float pgg = 0;
    if (bits == 12) {
      pgg = ((float)x_int)*span/4095.0 + offset;
    }
    if (bits == 16) {
      pgg = ((float)x_int)*span/65535.0 + offset;
    }
    return pgg;
};

//**************************************
/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60::AK60(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 22.0f;
    _motor_data->current_feedback_limit = _I_MAX;
    _V_MAX = 41.87f;
    
    float kt = 0.068 * 6;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK60::AK60 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60v1_1::AK60v1_1(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 13.5f;
    _motor_data->current_feedback_limit = _I_MAX;
    _V_MAX = 23.04f;

    // 我们将转矩常数KT取值为0.1725×6，该数值和电机厂商标称的KT参数不一致，原厂给出的参数存在偏差（该修正值已通过多种方式多次验证）。
    // 我们仅对本型号电机完成了参数校验，髋关节关节会使用该电机做开环控制；其余电机均采用闭环控制，运行时会实时修正转矩系数。
    // 若你要在开环控制场景下使用该电机，建议自行重新校验KT转矩常数。
    float kt = 0.1725 * 6; //We set KT to 0.1725 * 6 whcih differs from the manufacturer's stated KT, that's because they are wrong (This has been validated mulitple ways). We only have validated for this version as we use open loop at the hip with these, other motors are used with closed loop and thus are corrected in real-time. We recommend validating these KTs if using for open loop. 
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK60v1_1::AK60v1_1 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK80::AK80(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 24.0f;
    _motor_data->current_feedback_limit = _I_MAX;
    _V_MAX = 25.65f;

    float kt = 0.091 * 9;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK80::AK80 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK70::AK70(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 23.2f;
    _motor_data->current_feedback_limit = _I_MAX;
    _V_MAX = 15.5f;
    
    float kt = 0.13 * 10;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

    #ifdef MOTOR_DEBUG
        logger::println("AK70::AK70 : Leaving Constructor");
    #endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK60v3::AK60v3(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 10.3f;
    _motor_data->current_feedback_limit = _I_MAX;
    _V_MAX = 48.0f;

    float kt = 0.420*6;//corrected values
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK60v3::AK60v3 : Leaving Constructor");
#endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK45_36::AK45_36(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 6.5f;
    _motor_data->current_feedback_limit = _I_MAX;
    _V_MAX = 5.44f;

    float kt = 0.127f;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK45_36::AK45_36 : Leaving Constructor");
#endif
};

/*
 * Constructor for the motor
 * Takes the joint id and a pointer to the exo_data
 * Only stores the id, exo_data pointer, and if it is left (for easy access)
 */
AK45_10::AK45_10(config_defs::joint_id id, ExoData* exo_data, int enable_pin): //Constructor: type is the motor type
_CANMotor(id, exo_data, enable_pin)
{
    _I_MAX = 6.5f;
    _motor_data->current_feedback_limit = _I_MAX;
    _V_MAX = 18.85f;

    float kt = 0.127f;
    set_Kt(kt);
    exo_data->get_joint_with(static_cast<uint8_t>(id))->motor.kt = kt;

#ifdef MOTOR_DEBUG
    logger::println("AK45_10::AK45_10 : Leaving Constructor");
#endif
};


/*
 * Constructor for the PWM (Maxon) Motor.  
 * We are using multilevel inheritance, so we have a general motor type, which is inherited by the PWM (e.g. Maxon) or other type (e.g. Maxon) since models within these types will share communication protocols, which is then inherited by the specific motor model, which may have specific torque constants etc.
 */
MaxonMotor::MaxonMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin) //Constructor: type is the motor type
: _Motor(id, exo_data, enable_pin)
{
    JointData* j_data = exo_data->get_joint_with(static_cast<uint8_t>(id));
	
    #ifdef MOTOR_DEBUG
        logger::println("MaxonMotor::MaxonMotor: Leaving Constructor");
    #endif
};

void MaxonMotor::transaction(float torque)
{
    //Send data
    send_data(torque);

    //Only enable the motor when it is an active trial 
    master_switch();

	if (_motor_data->enabled)
	{
		maxon_manager(true); //Monitors for and corrects motor resetting error if the system is operational.
	}
	else
	{
		maxon_manager(false);   //Reset the motor error detection function, in case user pauses device in middle of error event
	}

	// Serial.print("\nRight leg MaxonMotor::transaction(float torque)  |  torque = ");
	// Serial.print(torque);
};

bool MaxonMotor::enable()
{
    return true;    //This function is currently bypassed for this motor at the moment.
};

bool MaxonMotor::enable(bool overide)
{	
	//Only change the state and send messages if the enabled state (used as a master switch for this motor) has changed.
    if ((_prev_motor_enabled != _motor_data->enabled) || overide)
    {
		if (_motor_data->enabled)   //_motor_data->enabled is controlled by the GUI
		{
            //Enable motor
			digitalWrite(_enable_pin,HIGH);         //Relocate in the future
		}

		_enable_response = true;
	}

	if (!overide)                   //When enable(false), send the disable motor command, set the analogWrite resolution, and send 50% PWM command
    {
		_enable_response = false;
		
        //Disable motor, the message after this shouldn't matter as the power is cut, and the send() doesn't send a message if not enabled.
		digitalWrite(_enable_pin,LOW);
		analogWrite(_ctrl_right_pin,_pwm_neutral_val);
		analogWrite(_ctrl_left_pin,_pwm_neutral_val);
    }
	
	if (!_motor_data->enabled)   //_motor_data->enabled is controlled by the GUI
		{
            //Disable motor
			digitalWrite(_enable_pin,LOW);         //Relocate in the future
		}

	_prev_motor_enabled = _motor_data->enabled;

    return _enable_response;
	
    #ifdef MOTOR_DEBUG
        logger::print(_prev_motor_enabled);
        logger::print("\t");
        logger::print(_motor_data->enabled);
        logger::print("\t");
        logger::print(_motor_data->is_on);
        logger::print("\n");
    #endif
};

void MaxonMotor::send_data(float torque) //Always send motor command regardless of the motor "enable" status
{
    #ifdef MOTOR_DEBUG
        logger::print("Sending data: ");
        logger::print(uint32_t(_motor_data->id));
        logger::print("\n");
    #endif
	
	int direction_modifier = _motor_data->flip_direction ? -1 : 1; 

	_motor_data->t_ff = torque;
    _motor_data->last_command = torque;
	
	uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) ||
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);
   
	if (_data->user_paused || !active_trial || _data->estop)        //Ignores the exo error handler for the moment
    {
        analogWrite(_ctrl_left_pin,_pwm_neutral_val);   //Set 50% PWM (0 current)
		analogWrite(_ctrl_right_pin,_pwm_neutral_val);	//Set 50% PWM (0 current)
    }
    else
    {
		//Constrain the motor pwm command
		uint16_t post_fuse_torque = max(_pwm_l_bound,_pwm_neutral_val+(direction_modifier*torque));    //Set the lowest allowed PWM command
		post_fuse_torque = min(_pwm_u_bound,post_fuse_torque);                              //Set the highest allowed PWM command
		analogWrite((_motor_data->is_left? _ctrl_left_pin : _ctrl_right_pin),post_fuse_torque);	//Send the motor command to the motor driver
	}
};

void MaxonMotor::master_switch()
{
   //Only run if the motor is supposed to be enabled
    uint16_t exo_status = _data->get_status();
    bool active_trial = (exo_status == status_defs::messages::trial_on) || 
        (exo_status == status_defs::messages::fsr_calibration) ||
        (exo_status == status_defs::messages::fsr_refinement);

	if (_data->user_paused || !active_trial || _data->estop)
    {
		pinMode(_err_left_pin, INPUT_PULLUP);
		pinMode(_err_right_pin, INPUT_PULLUP);
		pinMode(_current_left_pin,INPUT);
		pinMode(_current_right_pin,INPUT);
		analogWriteResolution(12);
		analogWriteFrequency(_ctrl_left_pin, 5000);
		analogWriteFrequency(_ctrl_right_pin, 5000);
		
		//_motor_data->enabled = false;
        enable(false);
    }
	else
    {
		//_motor_data->enabled = true;
        enable(true);
	}
};

//Our implementation of the Maxon motor including the ec motor and the Escon 50_8 Motor Controller would occasionally cause 50_8 to enter error mode, with "Over current" being one of the errors.
//To address this issue, we have developed a solution contained in maxon_manager() below. 
void MaxonMotor::maxon_manager(bool manager_active)
{
    //Initialize variables when switch is set to false, run the error detection and rest code when switch is set to true. 
    if (!manager_active)
    {
		//Reset Maxon motor reset utilities
        do_scan4maxon_err_left = true;       
        maxon_counter_active_left = false;
		do_scan4maxon_err_right = true;       
        maxon_counter_active_right = false;
    }
    else
    {
		unsigned long maxon_reset_current_t = millis();
        
		//Scan for left motor error
		if ((do_scan4maxon_err_left) && (!digitalRead(_err_left_pin)))
		{
			do_scan4maxon_err_left = false;          
			maxon_counter_active_left = true;
			zen_millis_left = maxon_reset_current_t;
		}

		//Left motor reset
		if (maxon_counter_active_left) 
		{
			//Two iterations after maxon_counter_actie = true, de-enable motor
			if (maxon_reset_current_t - zen_millis_left >= 2)
			{
				enable(false);
			}

			//Ten iterations after maxon_counter_actie = true, re-enable motor
			if (maxon_reset_current_t - zen_millis_left >= 10)
			{
				enable(true);
			}
			
			//Thirty iterations after maxon_counter_actie = true, start scanning for error again
			if (maxon_reset_current_t - zen_millis_left >= 30)
			{
				do_scan4maxon_err_left = true;
				maxon_counter_active_left = false;                                   
				_motor_data->maxon_plotting_scalar = -1 * _motor_data->maxon_plotting_scalar;
			}
		}

		//Scan for right motor error
		if ((do_scan4maxon_err_right) && (!digitalRead(_err_right_pin)))
		{
			do_scan4maxon_err_right = false;          
			maxon_counter_active_right = true;
			zen_millis_right = maxon_reset_current_t;
		}
		
		//Right motor reset
		if (maxon_counter_active_right) 
		{
			//Two iterations after maxon_counter_actie = true, de-enable motor
			if (maxon_reset_current_t - zen_millis_right >= 2)
			{
				enable(false);
			}

			//Ten iterations after maxon_counter_actie = true, re-enable motor
			if (maxon_reset_current_t - zen_millis_right >= 10)
			{
				enable(true);
			}
			
			//Thirty iterations after maxon_counter_actie = true, start scanning for error again
			if (maxon_reset_current_t - zen_millis_right >= 30)
			{
				do_scan4maxon_err_right = true;
				maxon_counter_active_right = false;                                   
				_motor_data->maxon_plotting_scalar = -1 * _motor_data->maxon_plotting_scalar;
			}
		}
    }
};


#endif
