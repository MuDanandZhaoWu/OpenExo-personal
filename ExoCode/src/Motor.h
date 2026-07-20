/**
 * @file Motor.h
 *
 * @brief Declares a class used to interface with motors
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/

#ifndef Motor_h
#define Motor_h

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)

#include "Arduino.h"

#include "ExoData.h"
#include "ParseIni.h"
#include "Board.h"
#include "Utilities.h"

#include <stdint.h>

/**
 * @brief 抽象基类，规定所有电机的统一接口规范      Abstract class to define the interface for all motors.
 * 所有电机控制器子类必须实现以下接口：     All controllers must have a:
 * void read_data()
 * void send_data(float torque)
 * void transaction(float torque)
 * void on_off()
 * bool enable()
 * bool enable(bool overide)
 * void zero()
 * bool get_is_left()
 * config_defs::joint_id get_id()
 */
class _Motor
{
	public:
		_Motor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        virtual ~_Motor(){};
		
        //Pure virtual functions, these will have to be defined for each one.
        
        /**
         * @brief Reads motor data from each motor used on that side and stores the values
         */
        virtual void read_data() = 0; 

        /**
         * @brief Sends the new motor command to the motor.
         * 
         * @param motor torque command in Nm
         */
		virtual void send_data(float torque) = 0;  
		
        /**
         * @brief Sends the new motor command to the motor and reads the current state of the motor.
         * 
         * @param motor torque command in Nm
         */
        /**
         * @brief 向电机下发最新控制指令，并读取电机当前运行状态
         *
         * @param torque 电机目标扭矩指令，单位：牛米(N·m)
         */
        virtual void transaction(float torque) = 0;
		
        /**
         * @brief Powers on or off the motors depending on the is_on value in motor data 
         */
        virtual void on_off() = 0;  
        
        /**
         * @brief Enables or disables the motors depending on the state stored in the corresponding enabled state in motor data.
         * Only sends commands if the state has changes in the motor data.
         */
        virtual bool enable() = 0;  
        
        /**
         * @brief Same as enable but will resend commands if override is true, regardless of what the state of the system is.
         */
        virtual bool enable(bool overide) = 0;  
        
        /**
         * @brief Set position to zero
         */
        virtual void zero() = 0;  
        
        /**
         * @brief Lets you know if it is a left or right side.
         * 
         * @return 1 if the motor is on the left side, 0 otherwise
         */
        virtual bool get_is_left();  
        
        /**
         * @brief Returns the motor id, same as the joint id
         *
         * @return the motor id
         */
        virtual config_defs::joint_id get_id();

        virtual float get_Kt() = 0;                 /**< Torque constant of the motor, at the motor output. [Nm/A] */

        virtual void set_error() = 0;               /**< Sets the error flag for the motor. */
		
	protected:
        config_defs::joint_id _id;                  /**< Motor ID */
		bool _is_left;
        ExoData* _data;
		MotorData* _motor_data;
        int _enable_pin;
        bool _prev_motor_enabled;       //记录“上一轮这个电机是否处于 enabled 状态”
        bool _prev_on_state;
        bool _error = false;
        float _Kt;                                  /**< 转矩常数   Torque constant of the motor, at the motor output. [Nm/A] */  
};

/**
 * @brief A motor that does nothing
 */
class NullMotor : public _Motor
{
    public:
    NullMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin):_Motor(id, exo_data, enable_pin) {};
    void read_data() {};
    void send_data(float torque) {};
    void transaction(float torque) {};
    void on_off() {};
    bool enable() {return true;};
    bool enable(bool overide) {return true;};
    void zero() {};
    float get_Kt() {return 0.0;};
    void set_error() {};
};

/**
 * @brief Class for Maxon EC motor
 */
class MaxonMotor : public _Motor
{
    public:
    MaxonMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
    void transaction(float torque);
	void read_data() {};
    void send_data(float torque);
    void on_off() {};
    bool enable();
    bool enable(bool overide);
    void zero() {};
    float get_Kt() {return 0.0;};
    void set_error() {};                        //Not yet implemented for this motor type
	void master_switch();
	void maxon_manager(bool manager_active);    /**< Quickly and automatically reset the Maxon motor in case of the driver board reporting an error. */
	
	protected:
	bool _enable_response;                           /**< 电机成功响应使能指令时，该变量为true, 标志通讯是否超时        True if the motor responded to an enable command */
	bool do_scan4maxon_err_left = true;              /**< Part of the Maxon motor driver error reporting utilities: A switch to enable or disable error detection */
	bool maxon_counter_active_left = false;          /**< Part of the Maxon motor driver error reporting utilities: A switch for the error detection counter */
	unsigned long zen_millis_left;                   /**< Part of the Maxon motor driver error reporting utilities: A timer for the motor reset function */
	bool do_scan4maxon_err_right = true;              /**< Part of the Maxon motor driver error reporting utilities: A switch to enable or disable error detection */
	bool maxon_counter_active_right = false;          /**< Part of the Maxon motor driver error reporting utilities: A switch for the error detection counter */
	unsigned long zen_millis_right;                   /**< Part of the Maxon motor driver error reporting utilities: A timer for the motor reset function */
	const int _ctrl_left_pin = logic_micro_pins::maxon_ctrl_left_pin;	/**< Teensy pin to transmit left Maxon motor pwm signals */
	const int _ctrl_right_pin = logic_micro_pins::maxon_ctrl_right_pin;	/**< Teensy pin to transmit right Maxon motor pwm signals */
	const int _err_left_pin = logic_micro_pins::maxon_err_left_pin;	/**< Teensy pin to receive left Maxon motor driver errors */
	const int _err_right_pin = logic_micro_pins::maxon_err_right_pin;	/**< Teensy pin to receive right Maxon motor driver errors */
	const int _current_left_pin = logic_micro_pins::maxon_current_left_pin;	/**< Teensy pin to receive left Maxon motor current data */
	const int _current_right_pin = logic_micro_pins::maxon_current_right_pin;	/**< Teensy pin to receive right Maxon motor current data */
	const int _pwm_neutral_val = logic_micro_pins::maxon_pwm_neutral_val;	/**< Neutral pwm command for Maxon motor drivers */
	const int _pwm_u_bound = logic_micro_pins::maxon_pwm_u_bound;	/**< Upper bound of pwm command for Maxon motor drivers */
	const int _pwm_l_bound = logic_micro_pins::maxon_pwm_l_bound;	/**< Lower bound of pwm command for Maxon motor drivers */
};


/**
 * @brief This will define some of the common communication used by all the CAN motors and should be inherited by all of them.
 */
class _CANMotor : public _Motor
{
    public:
        _CANMotor(config_defs::joint_id id, ExoData* exo_data, int enable_pin);
        virtual ~_CANMotor(){};
        void transaction(float torque);
        void read_data();
        void send_data(float torque);
        void on_off();
        bool enable();
        bool enable(bool overide);
        void zero();
        float get_Kt();
        void check_response();
        void set_error();
        
    protected:

        void set_Kt(float Kt);
        bool _queue_zero_command();
        
        /**
         * @brief 电机官方给出的数据类型转换函数            Packs a float into the uint format needed to be sent to the motor.
         *
         * @param Float to be packed
         * @param Lower limit of the range of x values, used for scaling
         * @param Upper limit of the range of x values, used for scaling
         * @param Number of bits to pack the value into, 12 or 16
         *
         * @return Should return a uint that has been scaled to a position between x_min and x_max.  Currently returns a float, but it seems to work.
         */
        float _float_to_uint(float x, float x_min, float x_max, int bits);
        
        /**
         * @brief 电机官方给出的数据类型转换函数        Unpacks a unsigned int format from the motor into a float.
         *
         * @param Unsigned int to be unpacked
         * @param Lower limit of the range of x values, used for scaling
         * @param Upper limit of the range of x values, used for scaling
         * @param Number of bits to pack the value into, 12 or 16
         *
         * @return unpacked float value 
         */
        float _uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
        
        /**
         * @brief 读取电机反馈数据失败时，处理通信超时问题      Detects timeouts in case of a read failure.
         *
         */
        void _handle_read_failure();
        
        float _KP_MIN;                              /**< 电机比例增益下限 - Lower limit of the P gain for the motor */
        float _KP_MAX;                              /**< 电机比例增益上限 - Upper limit of the P gain for the motor */
        float _KD_MIN;                              /**< 电机微分增益下限 - Lower limit of the D gain for the motor */
        float _KD_MAX;                              /**< 电机微分增益上限 - Upper limit of the D gain for the motor */
        float _P_MAX;                               /**< 电机最大角度 - Max angle of the motor */
        float _I_MAX;                               /**< 电机最大工作电流 -  Max current of the motor */
        float _V_MAX;                               /**< 电机最大转速 - Max velocity of the motor */
        bool _enable_response;                      /**< 标识电机是否成功响应使能指令；电机正常应答时该值为true       trueTrue if the motor responded to an enable command */
        const uint32_t _timeout = 500;              /**< 等待电机返回应答的超时时间，单位：微秒     Time to wait for response from the motor in micro-seconds */

        std::queue<float> _measured_current;        /**< 存储电机实测电流采样值的队列       Queue of the measured current values */
        const int _current_queue_size = 25;         /**< 队列长度定义       Size of the queue of measured current values */
        const float _variance_threshold = 0.01;     /**< 实测电流采样值的方差判定阈值       Threshold for the variance of the measured current values */
        bool _disable_pending = false;              /**< true：触发了电机停机流程，但完整停机流程还没走完，需要依次下发位置归零报文 + 电机禁用断电报文      Stop still needs zero and disable queued. */
        bool _disable_zero_queued = false;          /**< 停机流程所需的归零指令报文，已经送入发送队列等待下发       Zero frame for the pending stop entered TX. */
        bool _power_restore_pending = false;        /**< 一个标志初始化是否完成的标志位, 电机上电恢复后，必须完成前置初始化流程，才允许执行电机使能操作     Power-up must be conditioned before enable. */
};

/**
 * @brief Class for AK60 V1.0 motor
 */
class AK60 : public _CANMotor
{
    public:
        AK60(config_defs::joint_id id, ExoData* exo_data, int enable_pin); //Constructor: type is the motor type
		~AK60(){};
};

/**
 * @brief Class for AK60 V1.1 motor - Takes Current for Input
 * 
 * 这是我们当前所使用的电机 This is the motor we are currently using
 */
class AK60v1_1 : public _CANMotor
{
    public:
        AK60v1_1(config_defs::joint_id id, ExoData* exo_data, int enable_pin); //Constructor: type is the motor type
		~AK60v1_1(){};
};

/**
 * @brief Class for AK80 V1.0 motor
 */
class AK80 : public _CANMotor
{
    public:
        AK80(config_defs::joint_id id, ExoData* exo_data, int enable_pin); //Constructor: type is the motor type
		~AK80(){};   
};

/**
 * @brief Class for AK70 V1.0 motor
 */
class AK70 : public _CANMotor
{
    public:
        AK70(config_defs::joint_id id, ExoData* exo_data, int enable_pin); //Constructor: type is the motor type
        ~AK70(){};
};

/**
* @brief Class for AK60v3 motor
*/
class AK60v3 : public _CANMotor
{
  	public:
          AK60v3(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // Constructor: type is the motor type
          ~AK60v3(){};
};

/**
* @brief Class for AK45-36 motor
*/
class AK45_36 : public _CANMotor
{
  	public:
          AK45_36(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // Constructor: type is the motor type
          ~AK45_36(){};
};

/**
* @brief Class for AK45-10 motor
*/
class AK45_10 : public _CANMotor
{
  	public:
          AK45_10(config_defs::joint_id id, ExoData* exo_data, int enable_pin); // Constructor: type is the motor type
          ~AK45_10(){};
};

#endif
#endif
