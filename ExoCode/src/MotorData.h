/**
 * @file MotorData.h
 *
 * @brief Declares a class used to store data for motors to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/

#ifndef MotorData_h
#define MotorData_h

#include "Arduino.h"

#include "ParseIni.h"
#include "Board.h"

#include <stdint.h>

//Forward declaration
class ExoData;

class MotorData 
{
	public:
        MotorData(config_defs::joint_id id, uint8_t* config_to_send);
        
        /**
         * @brief reconfigures the the motor data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        config_defs::joint_id id;   /**< Motor id, should be the same as the joint id. */ 
        uint8_t motor_type;         /**< Type of motor being used. */
        float last_command;         /**< Last command sent to the motor. */
        float p;                    /**< Read position. */ 
        float v;                    /**< Read velocity. */
        float i;                    /**< Read current. */
        float kt;                   /**< 电机扭矩常数 - Motor torque constant. */
        float current_feedback_limit; /**< Absolute current-feedback encoding limit [A], not a wearable safety limit. */
        float p_des = 0;            /**< Desired position, not currently used but available for position control */
        float v_des = 0;            /**< Desired velocity, not currently used but available for velocity control */
        float kp = 0;               /**< Proportional gain */
        float kd = 0;               /**< Derivative gain */
        float t_ff = 0;             /**< Torque command */
        
        bool do_zero;               /**< Flag to zero the position of the motor */
        bool enabled;               /**< Motor enable state*/
        bool is_on;                 /**< Motor power state */
        bool is_left;               /**< Motor side information 1 if on the left, 0 otherwise */
        bool flip_direction;        /**< 是否反转电机转向；该标记为true时，扭矩指令、位置与速度反馈数据都会取反     Should the motor direction be flipped, if true torque commands and position/velocity information will be inverted */
        float gearing;              /**< Motor gearing used to convert motor position, velocity, and torque between the motor and joint frames. */

        //Timeout state
        int timeout_count = 0;      /**< Number of timeouts in a row */
        int timeout_count_max = 40; /**< Number of timeouts in a row before the motor is disabled */
        bool feedback_valid = false;        /**< 当成功接收并校验该电机的反馈报文后，本标识会置为 true      True after a validated response for this motor. */
        uint32_t last_feedback_us = 0;      /**< Timestamp of the last validated response. */
        uint32_t feedback_sequence = 0;     /**< 每收到一帧校验通过的反馈报文，该变量自增1      Incremented once for each validated response. */
		
		//Real-time Maxon Motor Reset Feedback
		int maxon_plotting_scalar = 1;

};


#endif
