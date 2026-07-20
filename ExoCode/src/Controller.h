/**
 * @file Controller.h
 *
 * @brief 声明外骨骼可使用的各类控制器  Declares for the different controllers the exo can use. 
 *        所有控制器均需继承 _Controller 类，以保证接口统一      Controllers should inherit from _Controller class to make sure the interface is the same.
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/

#ifndef Controller_h
#define Controller_h

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include <Arduino.h>

#include "ExoData.h"
#include "Board.h"
#include "ParseIni.h"
#include <stdint.h>
#include "Utilities.h"
#include "config.h"
#include "Time_Helper.h"
#include <algorithm>
#include <utility>

#include <Adafruit_INA260.h>

/**
 * @brief 该类定义控制器的接口规范  This class defines the interface for controllers.  
 * 所有控制器都必须实现 float calc_motor_cmd() 方法，该方法返回以牛米（Nm）为单位的力矩控制指令All controllers must have a: float calc_motor_cmd() that returns a torque cmd in Nm.  
 * 
 */
class _Controller
{
	public:
        /**
         * @brief Constructor 
         * 
         * @param id of the joint being used
         * @param pointer 指向完整 ExoData 实例的指针   to the full ExoData instance
         */
        _Controller(config_defs::joint_id id, ExoData* exo_data);
		
        /**
         * @brief 虚析构函数，确保删除派生类对象时调用正确的析构函数
         *        即保证用基类指针删除派生类对象时，派生类的析构函数也能被正确调用，避免内存泄漏 / 资源泄漏        Virtual destructor is needed to make sure the correct destructor is called when the derived class is deleted.
         */
        virtual ~_Controller(){};
        
        /**
         * @brief 纯虚函数，要求每个派生控制器必须实现计算电机控制指令的函数    Virtual function so that each controller must create a function that will calculate the motor command
         * 
         * @return 力矩，单位：牛米     Torque in Nm.
         */
		virtual float calc_motor_cmd() = 0; 
        
        /**
         * @brief 重置控制器的积分累加和     Resets the integral sum for the controller
         */
        void reset_integral(); 
        
    protected:
        
        ExoData* _data;                     /**< Pointer to the full data instance*/
        ControllerData* _controller_data;   /**< Pointer to the data associated with this controller */
        SideData* _side_data;               /**< Pointer for the side data the controller is associated with */
        JointData* _joint_data;             /**< Pointer to the joint data the controller is associated with */
         
        config_defs::joint_id _id;          /**< 该控制器所绑定的关节ID     Id of the joint this controller is attached to. */
        
        Time_Helper* _t_helper;             /**< 时间辅助工具实例，用于记录事件发生时间，判断PID是否到达设定执行时间    Instance of the time helper to track when things happen used to check if we have a set time for the PID */
        float _t_helper_context;            /**< 存储时间辅助工具的上下文状态   Store the context for the timer helper */
        float _t_helper_delta_t;            /**< 距上一次事件的时间间隔     Time time since the last event */

        //PID控制器相关变量     Values for the PID controller
        float _pid_error_sum = 0;           /**< 用于计算积分项的误差累加和     Summed error term for calucating intergral term */
        float _prev_input;                  /**< 上一时刻误差值，用于计算微分项     Prev error term for calculating derivative */
        float _prev_de_dt;                  /**< 上一时刻误差微分值，用于时间步长异常时备用     Prev error derivative used if the timestep is not good*/
        float _prev_pid_time;               /**< 上一次执行PID计算的时间        Prev time the PID was called */

        float _sim_gait_context = 0.0f;     /**< 模拟步态的计时上下文, 记录步态模拟的计时起点，用于在没有实际步态传感器数据时模拟步态百分比      Timer context for simulated gait */
        float _sim_elapsed_us = 0.0f;       /**< 记录自步态模拟开始以来已经过去的模拟时间（微秒）Accumulated simulated time */
        		
        /**
         * @brief 计算当前 PID 对电机控制指令的输出分量     Calculates the current PID contribution to the motor command. 
         * 
         * @param controller command(期望值/设定值）  
         * @param measured controlled value（实际测量值）     
         * @param proportional gain     比例增益
         * @param integral gain         积分增益
         * @param derivative gain       微分增益
         */
        float _pid(float cmd, float measurement, float p_gain, float i_gain, float d_gain);

        /**
         * @brief 获取步态百分比，可选择使用 1 秒周期的模拟步态     Returns percent gait, optionally using a simulated 1-second cycle.
         *
         * @param simulate 是否使用模拟步态百分比的标志位   flag to use simulated percent gait
         */
        float _get_percent_gait(bool simulate);
		
		/**
         * @brief A function that returns cmd_ff for stateless PJMC. 为无状态（没记忆、不存历史、每次只看当下输入、独立计算） PJMC 算法计算前馈控制量 cmd_ff
         * 
         * @param current 校准后的当前 FSR 力敏电阻百分比值，正常范围 0~1   fsr percentage value (after calibration, this value should typical range from 0 to 1 
         * @param fsr FSR 阈值；当前 FSR 值低于此阈值时，函数将返回对应负向设定值的前馈量   threshold; a current fsr value below this threshold will have the generic pjmc function return a cmd_ff with a sign of setpoint_negative
         * @param setpoint_positive 正向设定值（当FSR值等于阈值时的输出）（符号遵循 OpenSim 默认模型：踝关节背屈、膝关节伸展、髋关节屈曲为正）  setpoint (the sign definitions follow those as shown in OpenSim's default models: Positive for dorsiflexion, knee extension, and hip flexion.
         * @param setpoint_negative 负向设定值（当FSR值达到最大值1(压力超过阈值)时的输出, 实际应用中可能需要对输出值进行限制以确保系统安全）
         * setpoint_positive与setpoint_negative分别定义了线性变换的两个端点
         */
        float _pjmc_generic(float current_fsr, float fsr_threshold, float setpoint_positive, float setpoint_negative);
        
        // 紧凑型无模型自适应控制器（Compact Form Model Free Adaptive Controller）相关参数  Values for the Compact Form Model Free Adaptive Controller
        std::pair<float, float> measurements;   // 测量值（存储当前/历史测量数据）
        std::pair<float, float> outputs;        // 输出值（存储控制器输出指令）
        std::pair<float, float> phi;            /**< 伪偏导数（Psuedo partial derivative，MFAC核心估计参数）用于近似系统的动态特性  Psuedo partial derivative */
        float rho;                              /**< 惩罚因子（取值范围 (0,1), 控制对控制输入变化的惩罚程度，用于平衡跟踪性能和控制输入平滑性   Penalty factor (0,1) */
        float lamda;                            /**< 加权因子（用于限制控制量增量Δu的幅值）限制控制输入的变化幅度（delta u），影响控制器的平滑性     Weighting factor limits delta u */
        float etta;                             /**< 步长常数（取值范围 (0, 1]）控制参数更新的速度    Step size constant (0, 1] */
        float mu;                               /**< 加权因子（用于限制控制量u的方差）影响控制的稳定性    Weighting factor that limits the variance of u */
        float upsilon;                          /**< 极小常数（推荐取值约为10的-5次方）用于防止数值奇异或除零错误    A sufficiently small integer ~10^-5 */
        float phi_1;                            /**< 伪偏导数估计的初始/重置条件值, 用于估计伪偏导数的初始值  Initial/reset condition for estimation of psuedo partial derivitave */
        
        float _cf_mfac(float reference, float current_measurement);
};

/**
 * @brief Terrain Responsive Exoskeleton Controller (TREC)
 * This controller is for the ankle joint
 *
 * For full details see: "Mixed Terrain Ankle Assistance and Modularity in Wearable Robotics" by Chancelor Frank Cuddeback (https://biomech.nau.edu/)
 *
 * See ControllerData.h for details on the parameters used.
 */
/**
    @brief 地形自适应外骨骼控制器（TREC）
    该控制器适用于踝关节

    控制器所用参数的详细说明参见 ControllerData.h。
*/
class TREC : public _Controller
{
public:
    TREC(config_defs::joint_id id, ExoData* exo_data);
    ~TREC() {};

    float calc_motor_cmd();

private:
    void _update_reference_angles(SideData* side_data, ControllerData* controller_data, float percent_grf, float percent_grf_heel);
    void _capture_neutral_angle(SideData* side_data, ControllerData* controller_data);
    void _grf_threshold_dynamic_tuner(SideData* side_data, ControllerData* controller_data, float threshold, float percent_grf_heel);
    void _plantar_setpoint_adjuster(SideData* side_data, ControllerData* controller_data, float pjmcSpringDamper);
};

/**
 * @brief Proportional Joint Moment Controller
 * This controller is for the ankle joint 
 * Applies a plantar torque based on the normalized magnitude of the toe FSR.
 * 
 * This controller is based on:
 * Gasparri, G.M., Luque, J., Lerner, Z.F. (2019).
 * Proportional Joint-Moment Control for Instantaneosuly Adaptive Ankle Exoskeleton Assistnace. IEEE TNSRE, 27(4), 751-759.
 *
 * See ControllerData.h for details on the parameters used.
 */
    /**
    @brief 比例关节力矩控制器
    该控制器适用于踝关节
    根据脚趾力敏电阻（FSR）的归一化幅值输出跖屈力矩。
    本控制器基于以下文献实现：

    控制器所用参数的详细说明参见 ControllerData.h。
    */
class ProportionalJointMoment : public _Controller
{
    public:
        ProportionalJointMoment(config_defs::joint_id id, ExoData* exo_data);
        ~ProportionalJointMoment(){};
        
        float calc_motor_cmd();
    private:
        std::pair<float, float> _stance_thresholds_left, _stance_thresholds_right;
        
        float _inclination_scaling{1.0f};
};


/**
 * @brief Zero Torque Controller
 * This controller is for the any joint
 * Simply applies zero torque
 * 
 * See ControllerData.h for details on the parameters used.
 */
/**
    @brief 零力矩控制器
    适用于任意关节
    仅输出零力矩
    控制器所用参数的详细说明参见 ControllerData.h。

    安全模式：当系统需要停止对关节施加任何控制力矩时，可以切换到此模式
    待机状态：在某些测试或待机状态下，确保关节不受额外力矩影响
    故障保护：在紧急情况或系统故障时，可切换至此模式以确保安全
    对比基准：在实验中作为对照组，对比有无外骨骼辅助的情况
*/
class ZeroTorque : public _Controller
{
    public:
        ZeroTorque(config_defs::joint_id id, ExoData* exo_data);
        ~ZeroTorque(){};
        
        float calc_motor_cmd();
};

/**
 * @brief Zhang Collins Controller
 * This controller is for the ankle joint 
 * Applies ramp between t0 and t1 to (t1, ts).
 * From t1 to t2 applies a spline going up to (t2,mass*normalized_peak_torque).
 * From t2 to t3 falls to (t3, ts)
 * From t3 to 100% applies zero torque
 * 
 * This controller is based on:
 * Zhang, J., Fiers, P., Witte, K. A., Jackson, R. W., Poggensee, K. L., Atkeson, C. G., & Collins, S. H. 
 * (2017). Human-in-the-loop optimization of exoskeleton assistance during walking. Science, 356(6344), 1280-1284.
 * 
 * See ControllerData.h for details on the parameters used.
 */
/**
    @brief 张 - 柯林斯控制器
    该控制器适用于踝关节
    在 t0 到 t1 阶段输出斜坡力矩，直至 (t1, ts)；
    在 t1 到 t2 阶段通过样条曲线输出力矩，峰值达到 (t2, 体重 × 归一化峰值力矩)；
    在 t2 到 t3 阶段力矩回落至 (t3, ts)；
    在 t3 到 100% 步态周期内输出零力矩。

    控制器所用参数的详细说明参见 ControllerData.h。
*/
class ZhangCollins: public _Controller
{
    public:
        ZhangCollins(config_defs::joint_id id, ExoData* exo_data);
        ~ZhangCollins(){};
        
        float calc_motor_cmd();

        float _spline_generation(float node1, float node2, float node3, float torque_magnitude, float percent_gait);

        float torque_cmd;
		float cmd;
};

/**
 * @brief Spline Controller
 * This controller is for the hip and ankle joints
 * Applies a spline curve defined by five (percent gait, torque) nodes.
 *
 * See ControllerData.h for details on the parameters used.
 */
/**
    @brief 样条控制器
    该控制器适用于髋关节与踝关节
    根据五个（步态百分比，力矩）节点定义的样条曲线输出力矩。

    控制器所用参数的详细说明参见 ControllerData.h。
*/
class Spline: public _Controller
{
    public:
        Spline(config_defs::joint_id id, ExoData* exo_data);
        ~Spline(){};

        float calc_motor_cmd();

    private:
        float _spline_interpolate(const float* x, const float* y, float percent_gait);
};

/**
 * @brief Franks Collins Controller
 * This controller is for the Hip Joint
 *
 * Is 0 between t0_trough and t1_trough to (t1, 0).
 * From t1_trough to t2_trough applies a spline going down to (t2_trough,mass*normalized_trough_torque).
 * From t2_trough to t3_trough rises to (t3_trough, 0)
 * From t3_trough to t1_peak applies zero torque
 * From t1_peak to t2_peak applies a spline going up to (t2_peak,mass*normalized_peak_torque).
 * From t2_peak to t3_peak falls to (t3_peak, 0)
 *
 * This controller was based on:
 * Franks, P. W., Bryan, G. M., Martin, R. M., Reyes, R., Lakmazaheri, A. C., & Collins, S. H.
 * (2021). Comparing optimized exoskeleton assistance of the hip, knee, and ankle in single and multi-joint configurations. Wearable Technologies, 2.
 *
 * See ControllerData.h for details on the parameters used.
 */
/**
@brief 弗兰克斯 - 柯林斯控制器
该控制器适用于髋关节，三段式助力

在 t0_trough 至 t1_trough 阶段力矩保持为 0，直至 (t1, 0)；
在 t1_trough 至 t2_trough 阶段通过样条曲线输出力矩，直至波谷点 (t2_trough, 体重 × 归一化波谷力矩)；
在 t2_trough 至 t3_trough 阶段力矩回升至 (t3_trough, 0)；
在 t3_trough 至 t1_peak 阶段输出零力矩；
在 t1_peak 至 t2_peak 阶段通过样条曲线输出力矩，直至峰值点 (t2_peak, 体重 × 归一化峰值力矩)；
在 t2_peak 至 t3_peak 阶段力矩回落至 (t3_peak, 0)。

本控制器基于以下文献实现：
Franks, P. W., Bryan, G. M., Martin, R. M., Reyes, R., Lakmazaheri, A. C., & Collins, S. H.
(2021). 单关节与多关节构型下髋、膝、踝关节外骨骼优化助力对比研究.
《可穿戴技术》

控制器所用参数的详细说明参见 ControllerData.h, 该控制器仍在开发中...
*/
class FranksCollinsHip: public _Controller
{
    public:
        FranksCollinsHip(config_defs::joint_id id, ExoData* exo_data);
        ~FranksCollinsHip(){};
       
        float calc_motor_cmd();
        //确保曲线在节点处平滑连接，避免输出的突变
        //在三个节点（预定义的步态相位点）之间创建平滑的三次样条曲线，每个节点区间内，函数使用三次多项式进行插值
        //三个节点分别为力矩开始上升的时间点，力矩达到峰值的时间点和力矩下降到零的时间点
        float _spline_generation(float node1, float node2, float node3, float torque_magnitude, 
            float shifted_percent_gait);

        float last_percent_gait;
        float last_start_time;
       
};

/**
 * @brief Constant Torque Controller
 * This controller is for any joint
 * Applies a constant torque, filter applied when changing magnitude of torque
 *
 * See ControllerData.h for details on the parameters used.
 */
/**
    @brief 恒定力矩控制器
    适用于任意关节
    输出恒定力矩，在力矩幅值变化时会启用滤波处理

    控制器所用参数的详细说明参见 ControllerData.h。
*/
class ConstantTorque : public _Controller
{
public:
    ConstantTorque(config_defs::joint_id id, ExoData* exo_data);
    ~ConstantTorque() {};

    float calc_motor_cmd();

    float previous_command;         /* Stores Previous Loop's Torque Command */
    float previous_torque_reading;  /* Stores Previous Loop's Measured Torque */
    int flag;                       /* Flag that Determines Filter Status */
    float difference;               /* Stores Difference in Command when Changed */

};

/**
 * @brief Elbow Controller 
 * This controller is for the elbow joint
 * Applies flexion or extension torque based on hand FSRs that assists with lifting motion 
 * 
 * This controller is detailed in:
 * Colley, D., Bowersock, C.D., Lerner, Z.F. (2024)
 * A Lightweight Powered Elbow Exoskeleton for Manual Handling Tasks. IEEE T-MRB, 6(4), 1627-1636.
 *
 * See ControllerData.h for details on the parameters used.
 */
/**
 * @brief 肘关节控制器
 * 该控制器适用于肘关节
 * 基于手部力敏电阻（FSR）信号输出屈肘或伸肘力矩，辅助完成抬举动作
 * 
 * 该控制器的详细设计参考：
 * Colley, D., Bowersock, C.D., Lerner, Z.F. (2024)
 * 《轻量化主动肘关节外骨骼在人工搬运任务中的应用》，IEEE 医用机器人与生物力学汇刊（T-MRB），第6卷第4期，1627-1636页.
 *
 * 控制器所用参数详情参见 ControllerData.h 文件。
 */
class ElbowMinMax : public _Controller
{
public:
    ElbowMinMax(config_defs::joint_id id, ExoData* exo_data);
    ~ElbowMinMax() {};

    float alpha0;
    float alpha1;
    float alpha2;
    float alpha3;

    float cmd;

    float Smoothed_Sig_Flex;
    float Smoothed_Sig_Ext;
    float Smoothed_Flex_Max;
    float Smoothed_Flex_Min;
    float Smoothed_Ext_Max;
    float Smoothed_Ext_Min;

    float starttime;

    float check;

    float Angle_Max;
    float Angle_Min;
    float Angle;

    bool flexState;
    bool extState;
    bool nullState;

    float previous_setpoint;

    float fsr_toe_previous_elbow;
    float fsr_heel_previous_elbow;

    float SpringEffect;

    float calc_motor_cmd();
    
};

/**
 * @brief Calibration Controller
 * This controller is for any joint.
 * Applies a constant torque to help calibrate direction sign (+/-) and ensure torque sensor sign (+/-) matches desired direction.
 * This controller should be utilized when first testing a new device, especially when incorporating a torque sensor.
 * Failure to do so can lead to amplification of error between desired and measured torque which can be unsafe. 
 *
 * See ControllerData.h for details on the parameters used. See documentation on procedures for calibration
 */
/**
    @brief 校准控制器
    适用于任意关节。
    输出恒定力矩，用于校准力矩方向的正负符号，确保力矩传感器的正负方向与预期方向一致。
    该控制器应在设备首次测试时使用，尤其在搭载力矩传感器的场景下。
    若未执行此校准，可能导致期望力矩与实测力矩之间的误差被放大，存在安全风险。
    控制器所用参数详见 ControllerData.h，校准流程说明参见相关文档。
*/
class CalibrManager : public _Controller
{
public:
    CalibrManager(config_defs::joint_id id, ExoData* exo_data);
    ~CalibrManager() {};

    float calc_motor_cmd();
};

/**
 * @brief Chirp Controller
 * This controller is for any joint
 * Applies a sinewave with user defined parameters. 
 * Used for hardware performance validation. 
 *
 * See ControllerData.h for details on the parameters used.
 */
/**
    @brief 
    线性扫频控制器
    适用于任意关节
    根据用户设定参数输出正弦扫频信号
    用于硬件性能校验
    控制器相关参数详情参见 ControllerData.h。
*/
class Chirp : public _Controller
{
public:
    Chirp(config_defs::joint_id id, ExoData* exo_data);
    ~Chirp() {};

    float start_flag;               /* Flag that triggers recording of the initial start time of the controller upon usage. */
    float start_time;               /* Variable that stores the start time of the controller. */
    float current_time;             /* Variable that stores the current time of the controller. */
    float previous_amplitude;       /* Variable that stores the previous amplitude, used as a switch to restart the controller if needed. (Set amplitude to 0 and then set to desired amplitude). */

    float calc_motor_cmd();         /* Function that calculates the motor command. */

};

/**
 * @brief Step Controller
 * This controller is for any joint
 * Applies step response to hardware of user specified magnitude, duration, and frequency.
 * Used for hardware performance validation.
 *
 * See ControllerData.h for details on the parameters used.
 */
/**
    @brief 阶跃控制器
    该控制器适用于任意关节
    根据用户指定的幅值、持续时间和频率，向硬件输出阶跃响应信号
    用于硬件性能验证
    具体参数说明详见 ControllerData.h。
*/
class Step : public _Controller
{
public:
    Step(config_defs::joint_id id, ExoData* exo_data);
    ~Step() {};

    int n;                          /* 记录已执行的阶跃次数     Keeps track of how many steps have been performed. */
    int start_flag;                 /* 阶跃信号首次输出时，触发记录起始时间的标志位     Flag that triggers the recording of the time that the step is first applied. */
    float start_time;               /* 阶跃信号首次输出的时间   Time that the step was first applied. */
    float cmd_ff;                   /* 电机前馈控制指令     Motor command. */
    float previous_time;            /* 存储上一次循环迭代的时间     Stores time from previous iteration. */
    float end_time;                 /* 记录阶跃信号结束的时间   Records time that step ended. */

    float previous_command;
    float previous_torque_reading;
    int flag;
    float difference;
    float turn;
    float flag_time;
    float change_time;

    float calc_motor_cmd();         /* 计算电机控制指令     Function that calculates the motor command. */

};

/**
 * @brief Hip impedance controller with FSR gait-phase feed-forward.
 *
 * The controller uses motor-derived joint position and velocity feedback. It
 * does not use the torque-sensor PID loop and does not implement a current PID.
 * Returned commands are joint-side torque in Nm.
 */
/**
 * @brief 搭载FSR步态相位前馈的髋关节阻抗控制器
 *
 * 本控制器以电机解算输出的关节位置、速度作为反馈信号；
 * 不使用力矩传感器构成PID闭环，也未搭建电流PID控制环。
 * 输出的控制指令为关节侧力矩，单位：牛米(N·m)。
 */
class FsrHipPd : public _Controller
{
public:
    FsrHipPd(config_defs::joint_id id, ExoData* exo_data);
    ~FsrHipPd() {};

    float calc_motor_cmd();
    void reset();
    void deactivate();
    bool is_fault_latched() const { return _state == State::FAULT_LATCHED; }

private:
    // 控制器状态枚举
    enum class State : uint8_t
    {
        DISABLED,       // 控制器未启用，保持零力矩输出
        WAIT_FEEDBACK,  // 等待新的有效电机反馈，随后记录关节中立位置
        WAIT_GAIT,      // 已获得电机反馈，等待有效步态相位和着地事件
        RAMPING,        // 按设定时间将控制力矩从零逐渐增加到正常值
        ACTIVE,         // 正常控制状态，持续计算并输出关节力矩
        FAULT_LATCHED   // 故障锁存状态，故障已锁存，禁用电机并保持零输出，等待安全复位
    };

    // 故障原因枚举
    enum class FaultReason : uint8_t
    {
        NONE,               // 当前没有故障
        INVALID_PARAMETER,  // 控制参数或计算结果非法，例如出现 NaN/无穷值
        FEEDBACK_TIMEOUT,    // 在规定时间内未收到新的有效电机反馈
        GAIT_TIMEOUT,        // 步态相位无效或在规定时间内未检测到着地事件
        HARD_POSITION,      // 关节相对位置达到硬限位阈值
        HARD_VELOCITY,      // 关节速度达到硬限速阈值
        OVERCURRENT,        // 电机电流连续达到过流跳闸条件
        ESTOP               // 急停信号被触发
    };

    State _state;                 // 控制器当前所处的运行状态
    FaultReason _fault_reason;    // 最近一次锁存故障的原因
    float _neutral_position;      // 启动时记录的关节中立位置，用作相对位置和参考轨迹的基准
    float _filtered_velocity;     // 经过指数加权移动平均滤波的关节速度
    float _previous_command;      // 上一次实际输出的关节力矩，用于限制力矩变化速率
    float _parameter_snapshot[controller_defs::fsr_hip_pd::num_parameter]; // 参数快照，用于检测运行期间的参数变化
    uint32_t _state_entry_ms;     // 进入当前状态时的毫秒时间戳，用于状态超时和渐增计时
    uint32_t _last_ground_strike_ms; // 最近一次检测到足部着地事件的毫秒时间戳
    uint32_t _previous_command_us;   // 上一次计算力矩指令时的微秒时间戳，用于变化率限制
    uint32_t _feedback_wait_started_us; // 开始等待电机反馈时的微秒时间戳，排除等待前的旧反馈
    uint32_t _last_feedback_sequence;   // 上一次处理的电机反馈序号，用于识别新反馈数据
    uint16_t _overcurrent_count;        // 连续新反馈中达到过流阈值的次数
    bool _neutral_valid;                // 是否已从有效电机反馈中取得中立位置
    bool _parameter_snapshot_valid;     // 参数快照是否已初始化并可用于比较
    bool _last_fault_reset;             // 上一周期的故障复位电平，用于检测复位信号上升沿

    float _zero_output();
    bool _parameters_valid() const;
    bool _parameters_changed() const;
    void _capture_parameters();
    bool _feedback_fresh() const;
    bool _phase_valid() const;
    bool _safe_to_reset_fault() const;
    void _enter_state(State state);
    void _latch_fault(FaultReason reason);
    float _feedforward_torque(float percent_gait) const;
    float _reference_offset(float percent_gait) const;
    float _apply_soft_limits(float torque, float relative_position, float velocity) const;
    float _apply_slew_limit(float torque, uint32_t now_us);
};

/**
 * @brief Proportional Hip Moment Controller
 * This controller is for the hip joint
 * Applies a torque based on an estimate of the hip moment.
 *
 * NOTE: THIS CONTROLLER IS STILL UNDERDEVELOPMENT
 * 
 * See ControllerData.h for details on the parameters used.
 */
/**
 * @brief 比例髋关节力矩控制器
 * 本控制器适用于髋关节
 * 根据髋关节力矩的估算值输出相应扭矩。
 *
 * 注意：此控制器仍在开发中
 * 
 * 所用参数的详细说明请参见 ControllerData.h。
 */
class ProportionalHipMoment : public _Controller
{
public:
    ProportionalHipMoment(config_defs::joint_id id, ExoData* exo_data);
    ~ProportionalHipMoment() {};
    
    /* Note: Duration in this controller is in terms of number of iterations in that window, rather than as a time. */

    int state;                      /* Keeps track of what state we are in: State 1 - Mid-to-Late Swing (15% onward), State 2: Stance, State 3: Early-Swing (First 15%). */

    bool first_state2;              /* Flag to set variable values upon the first instance of the current State 2. */
    bool first_state3;              /* Flag to set variable values upon the first instance of the current State 3. */

    int swing_counter;              /* Keeps track of the number of iterations that have occured in current swing phase. */
    int state1_counter;             /* Keeps track of the number of iterations that have occured in current State 1. */
    int prev_state1_counter;        /* Stores the previous duration of State 1, used to estimate position in current State 1 relative to expected duration. */
    int stance_counter;             /* Keeps track of the number of iterations that have occured in current Stance Phase. */
    int swing_duration;             /* Stores the duration of the previous swing phase. */
 
    float setpoint;                 /* 存储计算得出的髋关节前馈控制目标值       Stores the calculated feed-foward setpoint for the hip command. */
    float old_setpoint;             /* 缓存状态3结束时刻的控制目标值，用于状态1下的目标值求解运算       Stores the setpoint at the end of State 3 to be used for setpoint calculation in State 1. */

    int state_count_12;             /* Keeps track of the number of iterations that have occured in the State 1 - to - State 2 Transition. */
    int state_count_23;             /* Keeps track of the number of iterations that have occured in the State 2 - to - State 3 Transition. */
    int state_count_31;             /* Keeps track of the number of iterations that have occured in the State 3 - to - State 1 Transition. */
    
    int Prev_latestance_duration;   /* Stores the previous duration of the late-stance phase (part of the late-stance to early-swing transition period. */
    int latestance_duration;        /* Records the duration of the recently ended late-stance phase. */
    int latestance_counter;         /* Keeps track of the number of iterations that have occured in the current Late-Stance Period. */
    float Alpha_counter;            /* Keeps track of the number of iterations that have occured in the current Late-Stance - and - Early Swing Transition Period. */
    float Alpha;                    /* Stores the expected duration of the Late-Stance - and - Early Swing Transition Period, based on the duration of the last transition period. */
    float t;                        /* Calculated percentage of stance-to-swing transition based on the duration of the previous stance-to-swing transition (expressed as 0.1, 0.2,... rather than 10%, 20%,...). */
    
    float fs;                       /* Ratio of heel and toe fsrs accounting for GRF Ratio (0.25). */
    float fs_min;                   /* Stores the minimum calculated fs for the current cycle, used as a starting estimate for the Late-Stance - to - Early Swing transition period. */
    float prev_fs;                  /* Stores the previous cycle's fs to help determine the slope of the fs curve. */
    float hip_ratio;                /* Part of calculation to determine the feed-foward setpoint calculation during stance-phase (State 2). */

    float calc_motor_cmd();         /* Function to calcualte the desired motor command. */

private:

};

/**
 * @brief SPV2 Controller
 * 
 * NOTE: THIS CONTROLLER IS STILL UNDER DEVELOPMENT 
 * 
 * See ControllerData.h for details on the parameters used.
 */
/**
    @brief SPV2 控制器

    注：该控制器仍处于开发阶段
    相关参数详情参见 ControllerData.h。
*/
class SPV2 : public _Controller
{
public:
    SPV2(config_defs::joint_id id, ExoData* exo_data);
    ~SPV2() {};
	Adafruit_INA260 ina260 = Adafruit_INA260();

    float calc_motor_cmd();

private:
	void SPV2::_plantar_setpoint_adjuster(SideData* side_data, ControllerData* controller_data, float currentPrescription);
	void SPV2::_stiffness_adjustment(uint8_t minAngle, uint8_t maxAngle, ControllerData* controller_data);
	void SPV2::_calc_motor_current(ControllerData* controller_data);
	void SPV2::_step_counter(uint16_t num_steps_threshold, SideData* side_data, ControllerData* controller_data);
	void SPV2::_golden_search_advance();
	void SPV2::optimizer_reset();
	void SPV2::_SA_point_gen(float step_size, long bound_l, long bound_u, float temp);
	void SPV2::_lab_OP_point_gen(float step_size, long bound_l, long bound_u);
	
	//from TREC
	void _update_reference_angles(SideData* side_data, ControllerData* controller_data, float percent_grf, float percent_grf_heel);
    void _capture_neutral_angle(SideData* side_data, ControllerData* controller_data);
    void _grf_threshold_dynamic_tuner(SideData* side_data, ControllerData* controller_data, float threshold, float percent_grf_heel);
    //void _plantar_setpoint_adjuster(SideData* side_data, ControllerData* controller_data, float pjmcSpringDamper);
};

/**
 * @brief PJMC_PLUS Controller
 * 
 * NOTE: THIS CONTROLLER IS STILL UNDER DEVELOPMENT 
 * 
 * See ControllerData.h for details on the parameters used.
 */
class PJMC_PLUS : public _Controller
{
public:
    PJMC_PLUS(config_defs::joint_id id, ExoData* exo_data);
    ~PJMC_PLUS() {};

    float calc_motor_cmd();

private:

};

#endif
#endif
