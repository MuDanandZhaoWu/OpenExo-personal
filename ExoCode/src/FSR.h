/**
 * @file FSR.h
 *
 * @brief Declares classes used to interface with a force sensitive resistor, note there is a regressed version and a non-regressed version
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef FSR_h
#define FSR_h

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "Arduino.h"
#include "Board.h"
#include "Utilities.h"
#include "ExoData.h"

/**
* @brief Handles raw (non-regressed) FSR signal. 
*   在原始 ADC 数值上进行校准，未进行力矩回归换算
*/
class FSR
{
	public:
		FSR(int pin);
		
        /**
         * @brief Does an initial time based calculation to find a rough range for the signal
         * Person should be walking
         * These values will be used for tracking the number of transitions for the calibration refinement.
         * 
         * @param if the calibration is active.
         * 
         * @return if the calibration is continuing.
         */
        /**
         * @brief 执行定时初始标定，获取传感器信号的大致幅值区间（粗校准）
         * 执行本标定时，使用者需要保持行走状态
         * 本次标定采集到的极值数据，将用于精细化标定时统计信号跳变次数
         *
         * @param do_calibrate 标定是否开启的使能标志
         *
         * @return 布尔值，代表当前标定流程是否仍在运行
         */
        bool calibrate(bool do_calibrate); 
		
        /**
         * @brief Does a refinement of the calibration based on averaging across an number of steps.
         * Person should be walking.
         * Refines the calibration, by finding the max and min over a set number of low to high transitions
         * 
         * @param if the calibration is active.
         * 
         * @return if the calibration is continuing.
         */
        /**
         * @brief 基于多步步态均值对标定区间做精细化校准
         * 执行该精细化校准期间，穿戴者需持续行走
         * 采集指定次数足底压力信号由低到高的状态跳变，提取每一步的最大、最小值，优化标定上下限
         *
         * @param do_refinement 精细化校准功能使能标志位
         *
         * @return 返回布尔值，代表精细化校准流程是否仍在执行中
         */
        bool refine_calibration(bool do_refinement);
        
        /**
         * @brief Reads the sensor and applies the calibration.  
         * If the refinement isn't done returns the regular calibration, 
         * If the regular calibration isn't done returns the raw value.
         * 
         * @return the sensor reading
         */
        /**
         * @brief 读取传感器采样值并执行标定换算
         * 若精细化标定未完成，则基于粗标定区间输出校准值；
         * 若粗标定也未执行，则直接返回传感器原始采样数据。
         * 
         * @return 经过标定处理后的传感器读数
         */
        float read(); //Reads the pins and updates the data object
		
        /**
         * @brief Uses a schmitt trigger to determine if the sensor is in contact with the ground (foot/shoe)
         * 
         * @return if the FSR is in contact with the ground
         */
        /**
         * @brief 采用施密特触发器算法判断压力传感器是否接触地面（足部/鞋底着地）
         * 
         * @return 压力传感器当前是否处于着地状态
         */
        bool get_ground_contact();

        /**
         * @brief Get the thresholds for the schmitt trigger
         * 
         * @param lower_threshold_percent_ground_contact lower threshold for the schmitt trigger
         * @param upper_threshold_percent_ground_contact upper threshold for the schmitt trigger
         */
        void get_contact_thresholds(float &lower_threshold_percent_ground_contact, float &upper_threshold_percent_ground_contact);
		
        /**
         * @brief Set the thresholds for the schmitt trigger
         * 
         * @param lower_threshold_percent_ground_contact lower threshold
         * @param upper_threshold_percent_ground_contact uppder threshold
         */
        void set_contact_thresholds(float lower_threshold_percent_ground_contact, float upper_threshold_percent_ground_contact);
	
    private:
		/**
         * @brief Calculates if the fsr is in contact with the ground based on a schmitt trigger
         * This is called in read()
         * 
         * @return if the sensor is in contact with the ground
         */
        bool _calc_ground_contact();  

        //Stores the sensor readings
        float _raw_reading;             /**< 传感器当前原始采样值       Current raw sensor reading */
		float _calibrated_reading;      /**< Sensor reading with calibration applied */
        
        int _pin;                       /**< The pin the sensor is connected to */
        
        //Used for calibration
        const uint16_t _cal_time = 5000;    /**< This is time to do the initial calibration */
        uint16_t _start_time;               /**< Stores the time we started the calibration */
        bool _last_do_calibrate;            /**< Used to find rising edge for calibration */
        float _calibration_min;             /**< Minimum value during the time period */
        float _calibration_max;             /**< Maximum value during the time period */
        
        //Used for calibration refinement
        const uint8_t _num_steps = 7;                                       /**< This is the number of steps to do the calibration_refinement */
        const float _lower_threshold_percent_calibration_refinement = .33;  /**< Lower threshold for the schmitt trigger. This can be relatively high since we don't really care about the exact moment the ground contact happens. */
        const float _upper_threshold_percent_calibration_refinement = .66;  /**< Upper threshold for the schmitt trigger */
        bool _state;        //当前FSR 状态。false 表示低状态，true 表示高状态。                                                        /**< Stores the signal high/low state from the schmitt trigger to find when there is a new step. */
        bool _last_do_refinement;       //上一周期的精细标定触发状态                                           /**< Used to track the rising edge of do_refinement, so we can reset on the first run. */
        unsigned int _step_max_sum;                                         /**< Stores the running sum of maximums from each step so we can average. */
        uint16_t _step_max;                                                 /**< Keeps track of the max value for the step. */
        unsigned int _step_min_sum;                                         /**< Stores the running sum of minimums from each step so we can average. */
        uint16_t _step_min;                                                 /**< Keeps track of the min value for the step. */
        uint8_t _step_count;                                                /**< Used to track if we have done the required number of steps. */
        float _calibration_refinement_min;                                  /**< The refined min used for doing the calibration */
        float _calibration_refinement_max;                                  /**< The refined max used for doing the calibration */
        
        //Used for ground_contact()
        bool _ground_contact;                                   /**< Is the FSR in contact with the ground */
        const uint8_t _ground_state_count_threshold = 4;        /**< Used to track if the FSR has been in contact with the ground for a while. */
        float _lower_threshold_percent_ground_contact = .15;    /**< Lower threshold for the schmitt trigger. This should be relatively low as we want to detect as close to ground contact as possible. */
        float _upper_threshold_percent_ground_contact = .25;    /**< Should be slightly higher than the lower threshold but by as little as you can get by with as the sensor must go above this value to register contact. */
};

/**
    * @brief Handles regressed FSR signal.
    * This is used for PJMC controller and is dependent on the type of FSR being used.
    * Current regression equation is for: Interlink 
    * 先把 ADC 数值经过经验公式转换成 Vo，然后再对 Vo 进行相同的校准
    */
class FSR_Regressed
{
	public:
		FSR_Regressed(int pin);
		
        /**
         * @brief Does an initial time based calculation to find a rough range for the signal
         * Person should be walking
         * These values will be used for tracking the number of transitions for the calibration refinement.
         * 
         * @param if the calibration is active.
         * 
         * @return if the calibration is continuing.
         */
        /**
         * @brief 执行基于固定时长的初始标定，获取传感器信号的大致幅值区间
         * 使用要求：使用者需保持行走状态
         * 本次标定得到的极值区间，会在精细化标定时用于统计信号状态跳变次数
         * 
         * @param 标定功能是否处于开启状态
         * 
         * @return 标定流程是否仍在运行中
         */
        bool calibrate(bool do_calibrate); 
		
        /**
         * @brief Does a refinement of the calibration based on averaging across an number of steps.
         * Person should be walking.
         * Refines the calibration, by finding the max and min over a set number of low to high transitions
         * 
         * @param if the calibration is active.
         * 
         * @return if the calibration is continuing.
         */
        bool refine_calibration(bool do_refinement);
        
        /**
         * @brief Reads the sensor and applies the calibration.  
         * If the refinement isn't done returns the regular calibration, 
         * if the regular calibration isn't done returns the raw value.
         * 
         * @return the sensor reading
         */
        float read(); //Reads the pins and updates the data object
		
        /**
         * @brief Uses a schmitt trigger to determine if the sensor is in contact with the ground (foot/shoe)
         * 
         * @return if the FSR is in contact with the ground
         */
        bool get_ground_contact();

        /**
         * @brief Get the thresholds for the schmitt trigger
         * 
         * @param lower_threshold_percent_ground_contact lower threshold for the schmitt trigger
         * @param upper_threshold_percent_ground_contact upper threshold for the schmitt trigger
         */
        void get_contact_thresholds(float &lower_threshold_percent_ground_contact, float &upper_threshold_percent_ground_contact);
		
        /**
         * @brief Set the thresholds for the schmitt trigger
         * 
         * @param lower_threshold_percent_ground_contact lower threshold
         * @param upper_threshold_percent_ground_contact uppder threshold
         */
        void set_contact_thresholds(float lower_threshold_percent_ground_contact, float upper_threshold_percent_ground_contact);
	
    private:
		/**
         * @brief Calculates if the fsr is in contact with the ground based on a schmitt trigger
         * This is called in read()
         * 
         * @return if the sensor is in contact with the ground
         */
        bool _calc_ground_contact();  

        //Stores the sensor readings
		float _raw_reading;         /**< Current raw sensor reading */
		float _calibrated_reading;  /**< Sensor reading with calibration applied */
        
        int _pin;                   /**< The pin the sensor is connected to. */
        
        //Used for calibration
        const uint16_t _cal_time = 5000;    /**< This is time to do the initial calibration */
        uint16_t _start_time;               /**< Stores the time we started the calibration */
        bool _last_do_calibrate;            /**< Used to find rising edge for calibration */
		float _calibration_min;             /**< Minimum value during the time period */
		float _calibration_max;             /**< Maximum value during the time period */
        
        //Used for calibration refinement
        const uint8_t _num_steps = 7;                                       /**< This is the number of steps to do the calibration_refinement */
        const float _lower_threshold_percent_calibration_refinement = .33;  /**< Lower threshold for the schmitt trigger. This can be relatively high since we don't really care about the exact moment the ground contact happens. */
        const float _upper_threshold_percent_calibration_refinement = .66;  /**< Upper threshold for the schmitt trigger */
        bool _state;                                                        /**< Stores the signal high/low state from the schmitt trigger to find when there is a new step. */
        bool _last_do_refinement;                                           /**< Used to track the rising edge of do_refinement, so we can reset on the first run. */
        unsigned int _step_max_sum;                                         /**< Stores the running sum of maximums from each step so we can average. */
        uint16_t _step_max;                                                 /**< Keeps track of the max value for the step. */
        unsigned int _step_min_sum;                                         /**< Stores the running sum of minimums from each step so we can average */
        uint16_t _step_min;                                                 /**< Keeps track of the min value for the step. */
        uint8_t _step_count;                                                /**< Used to track if we have done the required number of steps. */
        float _calibration_refinement_min;                                  /**< The refined min used for doing the calibration */
        float _calibration_refinement_max;                                  /**< The refined max used for doing the calibration */
        
        //Used for ground_contact()
        // FSR触底判断
        bool _ground_contact;                                   /**< Is the FSR in contact with the ground */
        // 状态计数防抖阈值：当计数器达到阈值时才最终判断为触底，用于判断FSR是否已经持续保持着地状态一段时间，过滤掉一些噪声与短暂的接触状态变化，提升触底检测的稳定性和可靠性。
        const uint8_t _ground_state_count_threshold = 4;        /**< Used to track if the FSR has been in contact with the ground for a while. */
        // 着地判定施密特触发器下限百分比阈值
        // 该值应设置得偏低，目的是尽可能精准捕捉足部刚接触地面的瞬间
        float _lower_threshold_percent_ground_contact = .15;    /**< Lower threshold for the schmitt trigger. This should be relatively low as we want to detect as close to ground contact as possible. */
        // 着地判定施密特触发器上限百分比阈值
        // 需略高于下限阈值，且两者差值尽量小；传感器读数必须超过该值，才会判定为有效着地
        float _upper_threshold_percent_ground_contact = .25;    /**< Should be slightly higher than the lower threshold but by as little as you can get by with as the sensor must go above this value to register contact. */
        
};
#endif
#endif