#include "Controller.h"
#include "Logger.h"

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include <math.h>

FsrHipPd::FsrHipPd(config_defs::joint_id id, ExoData* exo_data)
    : _Controller(id, exo_data)
{
    reset();
}

void FsrHipPd::reset()
{
    _state = State::DISABLED;
    _fault_reason = FaultReason::NONE;
    _neutral_position = 0.0f;
    _filtered_velocity = 0.0f;
    _previous_command = 0.0f;
    _state_entry_ms = millis();
    _last_ground_strike_ms = 0;
    _previous_command_us = 0;
    _feedback_wait_started_us = micros();
    _last_feedback_sequence = _joint_data->motor.feedback_sequence;
    _overcurrent_count = 0;
    _neutral_valid = false;
    _parameter_snapshot_valid = false;
    const float reset_value = _controller_data->parameters[controller_defs::fsr_hip_pd::fault_reset_idx];
    _last_fault_reset = isfinite(reset_value) && reset_value == 1.0f;
    _zero_output();
}

void FsrHipPd::deactivate()
{
    // A controller selection change must not provide a shortcut around a
    // latched safety fault. Normal deactivation, however, discards all state
    // so a later activation has to acquire a new neutral position and gait.
    // 切换控制器不能作为绕过锁存安全故障的捷径；但常规停用控制器操作会清空全部运行状态，
    // 因此下次重新启用该控制器时，需要重新获取关节中立位置、重新识别步态信息。
    if (_state == State::FAULT_LATCHED)
    {
        _zero_output();
        return;
    }
    //重置相关参数
    reset();
}

float FsrHipPd::_zero_output()
{
    _controller_data->ff_setpoint = 0.0f;
    _controller_data->desired_torque = 0.0f;
    _previous_command = 0.0f;
    _previous_command_us = 0;
    return 0.0f;
}

void FsrHipPd::_enter_state(State state)
{
    _state = state;
    _state_entry_ms = millis();
    if (state == State::WAIT_FEEDBACK)
    {
        _feedback_wait_started_us = micros();
        _last_feedback_sequence = _joint_data->motor.feedback_sequence;
    }
}

//故障锁存函数
void FsrHipPd::_latch_fault(FaultReason reason)
{
    _fault_reason = reason;
    _state = State::FAULT_LATCHED;
    _joint_data->motor.enabled = false;
    _joint_data->motor.is_on = false;
    _joint_data->motor.p_des = 0.0f;
    _joint_data->motor.v_des = 0.0f;
    _joint_data->motor.kp = 0.0f;
    _joint_data->motor.kd = 0.0f;
    _zero_output();
    logger::println(
        "FsrHipPd fault: joint=" + String((uint8_t)_id) +
        ", reason=" + String((uint8_t)_fault_reason),
        LogLevel::Error);
}

//进行参数合法性校验
bool FsrHipPd::_parameters_valid() const
{
    //p是指向参数数组parameters的指针，p[0]为参数数组的第一个元素，p[1]为第二个元素，依次类推。
    const float* p = _controller_data->parameters;
    for (uint8_t i = 0; i < controller_defs::fsr_hip_pd::num_parameter; ++i)
    {
        if (!isfinite(p[i]))
        {
            return false;
        }
    }

    const float enable = p[controller_defs::fsr_hip_pd::enable_idx];
    const float phase_1 = p[controller_defs::fsr_hip_pd::ff_phase_1_idx];
    const float phase_2 = p[controller_defs::fsr_hip_pd::ff_phase_2_idx];
    const float phase_3 = p[controller_defs::fsr_hip_pd::ff_phase_3_idx];
    const float ref_0 = p[controller_defs::fsr_hip_pd::reference_0_deg_idx];
    const float ref_50 = p[controller_defs::fsr_hip_pd::reference_50_deg_idx];
    const float kp = p[controller_defs::fsr_hip_pd::kp_idx];
    const float kd = p[controller_defs::fsr_hip_pd::kd_idx];
    const float alpha = p[controller_defs::fsr_hip_pd::velocity_alpha_idx];
    const float torque_limit = p[controller_defs::fsr_hip_pd::torque_limit_idx];
    const float slew_limit = p[controller_defs::fsr_hip_pd::torque_slew_limit_idx];
    const float soft_angle = p[controller_defs::fsr_hip_pd::soft_angle_deg_idx];
    const float hard_angle = p[controller_defs::fsr_hip_pd::hard_angle_deg_idx];
    const float soft_velocity = p[controller_defs::fsr_hip_pd::soft_velocity_idx];
    const float hard_velocity = p[controller_defs::fsr_hip_pd::hard_velocity_idx];
    const float current_warning = p[controller_defs::fsr_hip_pd::current_warning_idx];
    const float current_trip = p[controller_defs::fsr_hip_pd::current_trip_idx];
    const float trip_count = p[controller_defs::fsr_hip_pd::current_trip_count_idx];
    const float feedback_timeout = p[controller_defs::fsr_hip_pd::feedback_timeout_ms_idx];
    const float gait_timeout = p[controller_defs::fsr_hip_pd::gait_timeout_ms_idx];
    const float ramp_time = p[controller_defs::fsr_hip_pd::ramp_time_ms_idx];
    const float fault_reset = p[controller_defs::fsr_hip_pd::fault_reset_idx];

    if (!((enable == 0.0f) || (enable == 1.0f)) ||
        !((fault_reset == 0.0f) || (fault_reset == 1.0f)))
    {
        return false;
    }
    if (!(0.0f <= phase_1 && phase_1 < phase_2 && phase_2 < phase_3 && phase_3 < 100.0f))
    {
        return false;
    }
    if (kp < 0.0f || kd < 0.0f || alpha <= 0.0f || alpha > 1.0f)
    {
        return false;
    }
    if (torque_limit <= 0.0f || slew_limit <= 0.0f)
    {
        return false;
    }
    if (soft_angle <= 0.0f || hard_angle <= soft_angle ||
        fabsf(ref_0) > soft_angle || fabsf(ref_50) > soft_angle)
    {
        return false;
    }
    if (soft_velocity <= 0.0f || hard_velocity <= soft_velocity)
    {
        return false;
    }
    const float current_feedback_limit =
        _joint_data->motor.current_feedback_limit;
    const bool null_motor =
        _joint_data->motor.motor_type ==
            (uint8_t)config_defs::motor::NullMotor ||
        _joint_data->motor.motor_type ==
            (uint8_t)config_defs::motor::not_used;
    if (current_warning <= 0.0f || current_trip <= current_warning ||
        !isfinite(current_feedback_limit) ||
        (!null_motor && current_feedback_limit <= 0.0f) ||
        (current_feedback_limit > 0.0f &&
         current_trip > current_feedback_limit))
    {
        return false;
    }
    if (trip_count < 1.0f || trip_count > 1000.0f || floorf(trip_count) != trip_count)
    {
        return false;
    }
    // The firmware loop is nominally 2 ms. Keep the feedback timeout above
    // one loop plus CAN latency, and reject sub-millisecond time values that
    // would otherwise quantize to zero in the integer timestamp arithmetic.
    if (feedback_timeout < 5.0f || feedback_timeout > 60000.0f ||
        gait_timeout < 1000.0f || gait_timeout > 60000.0f ||
        ramp_time < 1.0f || ramp_time > 60000.0f)
    {
        return false;
    }

    //前馈扭矩参数索引数组
    const uint8_t torque_indices[] =
    {
        controller_defs::fsr_hip_pd::ff_torque_1_idx,
        controller_defs::fsr_hip_pd::ff_torque_2_idx,
        controller_defs::fsr_hip_pd::ff_torque_3_idx
    };
    for (uint8_t i = 0; i < 3; ++i)
    {
        //检查前馈力矩是否超过SD卡文件中的设定上限
        if (fabsf(p[torque_indices[i]]) > torque_limit)
        {
            return false;
        }
    }
    return true;
}

//判断FsrHipPd 的控制参数从上一次保存快照之后，是否发生了变化
bool FsrHipPd::_parameters_changed() const
{
    if (!_parameter_snapshot_valid)
    {
        return true;
    }
    for (uint8_t i = 0; i < controller_defs::fsr_hip_pd::fault_reset_idx; ++i)
    {
        if (_parameter_snapshot[i] != _controller_data->parameters[i])
        {
            //参数发生变化或还没有有效快照返回true
            return true;
        }
    }
    return false;
}

// 保存参数快照
void FsrHipPd::_capture_parameters()
{
    for (uint8_t i = 0; i < controller_defs::fsr_hip_pd::num_parameter; ++i)
    {
        _parameter_snapshot[i] = _controller_data->parameters[i];
    }
    _parameter_snapshot_valid = true;
}

// 用于判断当前电机反馈是否“有效且足够新”
bool FsrHipPd::_feedback_fresh() const
{
    // 创建电机数据的只读别名(只读引用, 在读取但并不改变源数据的场景下常用)
    const MotorData& motor = _joint_data->motor;
    //对电机反馈数据进行有效性检查(电机是否有有效的反馈, 位置是否finite, 速度是否finite, 电流是否finite)
    if (!motor.feedback_valid || !isfinite(_joint_data->position) ||
        !isfinite(_joint_data->velocity) || !isfinite(motor.i))
    {
        return false;
    }
    const uint32_t timeout_us = (uint32_t)
        (_controller_data->parameters[controller_defs::fsr_hip_pd::feedback_timeout_ms_idx] * 1000.0f);
    return (uint32_t)(micros() - motor.last_feedback_us) <= timeout_us;
}

// 检查percent_gait是否合规
bool FsrHipPd::_phase_valid() const
{
    return isfinite(_side_data->percent_gait) &&
        _side_data->percent_gait >= 0.0f && _side_data->percent_gait <= 100.0f;
}

//判断当前系统是否满足安全清除锁存故障（FAULT_LATCHED）的全部前置条件
bool FsrHipPd::_safe_to_reset_fault() const
{
    if (_data->get_status() != status_defs::messages::trial_off ||
        _data->estop || _joint_data->motor.enabled || !_parameters_valid() ||
        _controller_data->parameters[controller_defs::fsr_hip_pd::enable_idx] != 0.0f)
    {
        return false;
    }

    // A latched fault disables the drive, so its position/velocity/current
    // samples can be frozen and cannot be used as recovery evidence. Recovery
    // therefore requires an explicit stopped/disabled/reset sequence. The
    // next activation starts in WAIT_FEEDBACK and validates a new CAN sample
    // before locking a new neutral position.
    // 故障锁存会禁用电机驱动，此时位置/速度/电流采样数据会被冻结，无法作为系统恢复的有效依据。
    // 因此，系统恢复必须执行明确的 停机→禁用→复位 操作流程。
    // 下次重新激活控制器时，会先进入等待反馈状态，校验新的CAN总线采样数据有效后，
    // 再重新锁定关节中立位置。
    return true;
}

//前馈扭矩
float FsrHipPd::_feedforward_torque(float percent_gait) const   //输入当前步态百分比, const：该函数只读取控制器数据，不修改对象成员
{
    /*
    读取三个控制节点
    x1、x2、x3：步态相位，单位为 %。
    y1、y2、y3：对应相位的前馈力矩，单位为 N·m
    0 ≤ x1 < x2 < x3 < 100
    */
    const float* p = _controller_data->parameters;
    const float x1 = p[controller_defs::fsr_hip_pd::ff_phase_1_idx];
    const float y1 = p[controller_defs::fsr_hip_pd::ff_torque_1_idx];
    const float x2 = p[controller_defs::fsr_hip_pd::ff_phase_2_idx];
    const float y2 = p[controller_defs::fsr_hip_pd::ff_torque_2_idx];
    const float x3 = p[controller_defs::fsr_hip_pd::ff_phase_3_idx];
    const float y3 = p[controller_defs::fsr_hip_pd::ff_torque_3_idx];

    float x0;   // 区间起点相位
    float y0;   // 区间起点前馈力矩
    float xn;   // 区间终点相位
    float yn;   // 区间终点前馈力矩
    if (percent_gait < x1)
    {
        x0 = x3 - 100.0f; //表示上一步态周期的节点3, 即x3
        y0 = y3;
        xn = x1;
        yn = y1;
    }
    else if (percent_gait < x2)
    {
        x0 = x1;
        y0 = y1;
        xn = x2;
        yn = y2;
    }
    else if (percent_gait < x3)
    {
        x0 = x2;
        y0 = y2;
        xn = x3;
        yn = y3;
    }
    else
    {
        x0 = x3;
        y0 = y3;
        xn = x1 + 100.0f;   //表示下一节点的节点1, 即x1
        yn = y1;
    }
    // 计算两个角度之间的插值, blend 表示当前相位在选定区间中的位置
    /*
    blend = 0：正好位于起点
    blend = 0.5：位于区间中间
    blend = 1：正好位于终点

    blend 是当前两个节点之间的局部插值比例，不是整个步态周期的力矩变化比例
    力矩增大还是减小，取决于 yn - y0 的正负
    */
    const float blend = (percent_gait - x0) / (xn - x0);
    // 计算前馈力矩, 当前力矩 = 起点力矩 + 插值比例 × (终点力矩 - 起点力矩)
    return y0 + blend * (yn - y0);
}

/*
根据当前步态百分比，在两个参考角度 ref_0 和 ref_50 之间生成
一条平滑、周期连续的关节参考角度偏移，并将结果从度转换成弧度

“关节参考角度偏移”表示：相对于控制器启动时记录的中立位置，
当前步态阶段希望髋关节偏向哪个方向、偏多少角度

percent_gait: 当前步态相位，范围通常为 0～100%
*/
float FsrHipPd::_reference_offset(float percent_gait) const
{
    const float* p = _controller_data->parameters;
    //读取两个参考角度
    const float ref_0 = p[controller_defs::fsr_hip_pd::reference_0_deg_idx];    //步态处于 0% 或 100% 时的参考角度偏移
    const float ref_50 = p[controller_defs::fsr_hip_pd::reference_50_deg_idx];  //步态处于 50% 时的参考角度偏移
    /*
    计算周期性(平滑)混合系数blend
    步态相位	余弦输入	cos()	blend
     0%	           0	    1	    0
     25%	       π/2	    0	    0.5
     50%	       π	    -1  	1
     75%	       3π/2 	0	    0.5
     100%	       2π	    1	    0
    */
    const float blend = 0.5f * (1.0f - cosf(2.0f * PI * percent_gait / 100.0f));
    //返回两个角度之间的插值, 参考关节位置 = 启动中立位置 + 周期性角度偏移
    return (ref_0 + blend * (ref_50 - ref_0)) * PI / 180.0f;
}

// 软限位, 通过软件算法，限制关节的运动角度 / 位置，防止关节超出安全范围
//当关节位置或速度接近安全边界时，逐渐削弱那些会让关节继续向危险方向运动的力矩；制动或使关节返回安全区域的力矩则保留
/*
torque：准备输出的关节力矩，单位 N·m。
relative_position：关节当前位置相对中立位置的偏移，单位 rad。
velocity：当前关节速度，单位 rad/s。
*/
float FsrHipPd::_apply_soft_limits(float torque, float relative_position, float velocity) const
{
    const float* p = _controller_data->parameters;
    //软限位角度, 从soft_angle这里开始逐渐削弱向外力矩
    const float soft_angle = p[controller_defs::fsr_hip_pd::soft_angle_deg_idx] * PI / 180.0f;
    //硬限位角度, 到hard_angle这里时，向外力矩被削减到零；外层代码还会锁存硬位置故障
    const float hard_angle = p[controller_defs::fsr_hip_pd::hard_angle_deg_idx] * PI / 180.0f;      //弧度制
    //对relative_position取绝对值, abs_position为绝对位置
    const float abs_position = fabsf(relative_position);
    /*
    只有同时满足两个条件才削弱力矩：
    位置已经超过软阈值。
    力矩方向与位置方向相同, 会持续将关节推离软阈值
    */
    if (abs_position > soft_angle &&
        ((relative_position > 0.0f && torque > 0.0f) ||
         (relative_position < 0.0f && torque < 0.0f)))
    {
        const float scale = (hard_angle - abs_position) / (hard_angle - soft_angle);
        //constrain(待限制的值, 最小值, 最大值)
        torque *= constrain(scale, 0.0f, 1.0f);
    }

    //软限位速度, 从soft_velocity这里开始逐渐削弱向外力矩
    const float soft_velocity = p[controller_defs::fsr_hip_pd::soft_velocity_idx];
    const float hard_velocity = p[controller_defs::fsr_hip_pd::hard_velocity_idx];
    const float abs_velocity = fabsf(velocity);
    if (abs_velocity > soft_velocity &&
        ((velocity > 0.0f && torque > 0.0f) ||
         (velocity < 0.0f && torque < 0.0f)))
    {
        const float scale = (hard_velocity - abs_velocity) / (hard_velocity - soft_velocity);
        torque *= constrain(scale, 0.0f, 1.0f);
    }
    return torque;
}

/*
执行扭矩变化速率限制

它限制的不是力矩绝对大小，而是每秒最多允许变化多少 N·m

torque：本周期希望输出的目标力矩，单位 N·m。
now_us：当前微秒时间戳，通常由 micros() 提供。
返回值：经过变化率限制后的实际力矩指令。
*/
float FsrHipPd::_apply_slew_limit(float torque, uint32_t now_us)
{
    if (_previous_command_us == 0)
    {
        _previous_command_us = now_us;
        _previous_command = 0.0f;
    }
    //计算时间差
    const float elapsed_seconds = (float)((uint32_t)(now_us - _previous_command_us)) / 1000000.0f;
    /*
    计算本周期最多允许变化多少力矩,

    torque_slew_limit 的单位是：N·m/s
    乘以经过的秒数后：N·m/s × s = N·m
    */
    const float max_delta =
        _controller_data->parameters[controller_defs::fsr_hip_pd::torque_slew_limit_idx] * elapsed_seconds;
    //限制范围, -max_delta > torque - _previous_command > max_delta
    const float delta = constrain(torque - _previous_command, -max_delta, max_delta);
    //更新输出
    _previous_command += delta;
    //更新时间戳
    _previous_command_us = now_us;
    return _previous_command;
}

/*
函数执行:
故障处理
→ 检查系统是否允许运行
→ 等待有效电机反馈
→ 等待步态起点
→ 计算前馈 + PD 力矩
→ 执行各种安全限制
→ 返回最终关节目标力矩
*/
float FsrHipPd::calc_motor_cmd()
{
    const float* p = _controller_data->parameters;
    const float fault_reset_value = p[controller_defs::fsr_hip_pd::fault_reset_idx];
    const bool fault_reset = isfinite(fault_reset_value) && fault_reset_value == 1.0f;
    //检测 fault_reset 是否从 0 变成 1，也就是复位上升沿
    const bool fault_reset_rising = fault_reset && !_last_fault_reset;
    //更新_last_fault_reset标志
    _last_fault_reset = fault_reset;

    //判断当前是否处于锁存状态
    if (_state == State::FAULT_LATCHED)
    {
        //禁用电机
        _joint_data->motor.enabled = false;
        //只有检测到fault_reset的上升沿并且满足安全清除锁存故障（FAULT_LATCHED）的全部前置条件时才进行reset清除故障状态
        if (fault_reset_rising && _safe_to_reset_fault())
        {
            reset();
            _last_fault_reset = true;
        }
        return _zero_output();
    }
    //当前急停是默认禁止的
    if (_data->estop)
    {
        _latch_fault(FaultReason::ESTOP);
        return _zero_output();
    }

    const bool trial_active = _data->get_status() == status_defs::messages::trial_on;
    const bool controller_enabled = isfinite(p[controller_defs::fsr_hip_pd::enable_idx]) &&
        p[controller_defs::fsr_hip_pd::enable_idx] == 1.0f;
    //只有试验处于 trial_on、用户没有暂停、控制器参数 enable=1 时才允许运行
    if (!trial_active || _data->user_paused || !controller_enabled)
    {
        reset();
        _last_fault_reset = fault_reset;
        _capture_parameters();
        return _zero_output();
    }

    //检查参数合法性
    if (!_parameters_valid())
    {
        _latch_fault(FaultReason::INVALID_PARAMETER);
        return _zero_output();
    }

    if (!_joint_data->motor.enabled)
    {
        reset();
        _last_fault_reset = fault_reset;
        _capture_parameters();
        return _zero_output();
    }

    //检查参数是否改变(主要是在GUI中的参数修改)
    if (_parameters_changed())
    {
        _capture_parameters();
        _neutral_valid = false;
        _filtered_velocity = 0.0f;
        _overcurrent_count = 0;
        _last_ground_strike_ms = 0;
        _last_feedback_sequence = _joint_data->motor.feedback_sequence;
        _zero_output();
        _enter_state(State::WAIT_FEEDBACK);
        return 0.0f;
    }

    const uint32_t now_ms = millis();
    if (_state == State::DISABLED)
    {
        _enter_state(State::WAIT_FEEDBACK);
        return _zero_output();
    }

    if (_state == State::WAIT_FEEDBACK)
    {
        const MotorData& motor = _joint_data->motor;
        const bool feedback_arrived_after_wait =
            static_cast<int32_t>(motor.last_feedback_us - _feedback_wait_started_us) >= 0;
        if (_feedback_fresh() &&
            motor.feedback_sequence != _last_feedback_sequence &&
            feedback_arrived_after_wait)
        {
            _neutral_position = _joint_data->position;
            _filtered_velocity = _joint_data->velocity;
            _neutral_valid = true;
            _last_feedback_sequence = _joint_data->motor.feedback_sequence;
            _enter_state(State::WAIT_GAIT);
        }
        else if ((uint32_t)(now_ms - _state_entry_ms) >=
                 (uint32_t)p[controller_defs::fsr_hip_pd::feedback_timeout_ms_idx])
        {
            _latch_fault(FaultReason::FEEDBACK_TIMEOUT);
        }
        return _zero_output();
    }

    if (!_feedback_fresh())
    {
        _latch_fault(FaultReason::FEEDBACK_TIMEOUT);
        return _zero_output();
    }

    // These hard protections remain active while waiting for gait as well as
    // while producing torque. WAIT_GAIT commands zero, but the drive is still
    // enabled and must not remain enabled through a hard-limit condition.
    // 该硬限位保护逻辑，在等待步态阶段、正常输出力矩阶段均持续生效。
    // 虽然【等待步态】状态下控制器输出零力矩，但电机驱动依旧处于使能状态；
    // 一旦触发硬限位工况，绝对不能维持驱动使能，必须立刻保护停机。
    const float relative_position = _joint_data->position - _neutral_position;
    const float raw_velocity = _joint_data->velocity;
    const float hard_angle = p[controller_defs::fsr_hip_pd::hard_angle_deg_idx] * PI / 180.0f;
    if (fabsf(relative_position) >= hard_angle)
    {
        _latch_fault(FaultReason::HARD_POSITION);
        return _zero_output();
    }
    if (fabsf(raw_velocity) >= p[controller_defs::fsr_hip_pd::hard_velocity_idx])
    {
        _latch_fault(FaultReason::HARD_VELOCITY);
        return _zero_output();
    }

    const float abs_current = fabsf(_joint_data->motor.i);
    const bool new_feedback = _joint_data->motor.feedback_sequence != _last_feedback_sequence;
    if (new_feedback)
    {
        _last_feedback_sequence = _joint_data->motor.feedback_sequence;
        if (abs_current >= p[controller_defs::fsr_hip_pd::current_trip_idx])
        {
            if (_overcurrent_count < 65535)
            {
                ++_overcurrent_count;
            }
        }
        else
        {
            _overcurrent_count = 0;
        }
    }
    if (_overcurrent_count >=
        (uint16_t)p[controller_defs::fsr_hip_pd::current_trip_count_idx])
    {
        _latch_fault(FaultReason::OVERCURRENT);
        return _zero_output();
    }
    if (abs_current >= p[controller_defs::fsr_hip_pd::current_trip_idx])
    {
        return _zero_output();
    }

    //等待步态起点
    if (_state == State::WAIT_GAIT)
    {
        //即步态百分比有效，并检测到脚跟触地事件
        if (_phase_valid() && _side_data->ground_strike)
        {
            _last_ground_strike_ms = now_ms;
            _enter_state(State::RAMPING);
        }
        //超时判断
        else if ((uint32_t)(now_ms - _state_entry_ms) >=
                 (uint32_t)p[controller_defs::fsr_hip_pd::gait_timeout_ms_idx])
        {
            //锁存故障
            _latch_fault(FaultReason::GAIT_TIMEOUT);
        }
        return _zero_output();
    }
    //如果步态周期不合法, 触发步态超时并锁存
    if (!_phase_valid())
    {
        _latch_fault(FaultReason::GAIT_TIMEOUT);
        return _zero_output();
    }
    //发生一次触地事件, 更新时间戳
    if (_side_data->ground_strike)
    {
        _last_ground_strike_ms = now_ms;
    }
    //超时锁存
    if ((uint32_t)(now_ms - _last_ground_strike_ms) >=
            (uint32_t)p[controller_defs::fsr_hip_pd::gait_timeout_ms_idx])
    {
        _latch_fault(FaultReason::GAIT_TIMEOUT);
        return _zero_output();
    }

    //滤波函数中的alpha参数，用于控制滤波器的响应速度
    const float alpha = p[controller_defs::fsr_hip_pd::velocity_alpha_idx];
    //滤波
    _filtered_velocity = utils::ewma(raw_velocity, _filtered_velocity, alpha);
    const float phase = _side_data->percent_gait;
    //获得前馈力矩
    const float feedforward = _feedforward_torque(phase);
    //参考角度 = 
    const float theta_reference = _neutral_position + _reference_offset(phase);

    // Stable joint-space impedance feedback; do not add a second current PID.
    //目标力矩 =前馈力矩+ Kp ×（参考角度 - 实际角度） - Kd × 滤波速度
    float command = feedforward +
        p[controller_defs::fsr_hip_pd::kp_idx] * (theta_reference - _joint_data->position) -
        p[controller_defs::fsr_hip_pd::kd_idx] * _filtered_velocity;

    //检查command的合法性
    if (!isfinite(command))
    {
        _latch_fault(FaultReason::INVALID_PARAMETER);
        return _zero_output();
    }

    //限幅
    const float torque_limit = p[controller_defs::fsr_hip_pd::torque_limit_idx];
    command = constrain(command, -torque_limit, torque_limit);

    //在 RAMPING 状态下，力矩从 0 平滑增加到完整值，避免突然冲击
    if (_state == State::RAMPING)
    {
        const float ramp = constrain(
            (float)((uint32_t)(now_ms - _state_entry_ms)) /
                p[controller_defs::fsr_hip_pd::ramp_time_ms_idx],
            0.0f, 1.0f);
        command *= ramp;
        if (ramp >= 1.0f)
        {
            _enter_state(State::ACTIVE);
        }
    }
    //力矩变化率限制
    command = _apply_slew_limit(command, micros());

    // Safety envelopes are deliberately downstream of the symmetric slew
    // limiter. Torque reduction must take effect immediately when a joint
    // enters a soft limit or the measured current enters the warning band.
    //软角度和软速度限制
    command = _apply_soft_limits(command, relative_position, raw_velocity);
    if (abs_current > p[controller_defs::fsr_hip_pd::current_warning_idx])
    {
        const float current_scale =
            (p[controller_defs::fsr_hip_pd::current_trip_idx] - abs_current) /
            (p[controller_defs::fsr_hip_pd::current_trip_idx] -
             p[controller_defs::fsr_hip_pd::current_warning_idx]);
        command *= constrain(current_scale, 0.0f, 1.0f);
    }
    command = constrain(command, -torque_limit, torque_limit);
    if (!isfinite(command))
    {
        _latch_fault(FaultReason::INVALID_PARAMETER);
        return _zero_output();
    }

    // Continue subsequent slew calculations from the torque that actually
    // passed the safety envelopes, not from the rejected pre-limit value.
    _previous_command = command;
    _controller_data->ff_setpoint = feedforward;
    _controller_data->desired_torque = command;
    return command;
}

#endif
