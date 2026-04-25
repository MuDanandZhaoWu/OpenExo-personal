/**
 * @file SideData.h
 *
 * @brief Declares a class used to store data for side to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef SideData_h
#define SideData_h

#include "Arduino.h"

#include "JointData.h"
#include "ParseIni.h"
#include "Board.h"
#include "InclinationDetector.h"

#include <stdint.h>

//Forward declaration
class ExoData;

/**
 * @brief class to store information related to the side.
 * 
 */
class SideData {
	   
    public:
        SideData(bool is_left, uint8_t* config_to_send);
        
        /**
         * @brief Reconfigures the side data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        JointData hip;      /**< Data for the hip joint */
        JointData knee;     /**< Data for the knee joint */
        JointData ankle;    /**< Data for the ankle joint */
        JointData elbow;    /**< Data for the elbow joint */
        JointData arm_1;    /**< Data for the arm 1 joint */
        JointData arm_2;    /**< Data for the arm 2 joint */
        
        //步态周期相关参数, 用于量化当前走到步态的哪一个阶段
        float percent_gait;             /**< 基于基于足跟触地估算的步态百分比   Estimate of the percent gait based on heel strike */
        float expected_step_duration;   /**< 基于最近步时估算的下一步预期时长   Estimate of how long the next step will take based on the most recent step times */

        float percent_stance;           /**< 基于足跟触地与足趾离地估算的支撑相百分比   Estimate of the percent stance based on heel strike and toe off */
        float expected_stance_duration; /**< 基于最近支撑相时长估算的下一支撑相预期时长     Estimate of how long the next stance will take based on the most recent stance times */

        float percent_swing;            /**< 基于足趾离地与足跟触地估算的摆动相百分比   Estimate of the percent swing based on toe off and heel strike */
        float expected_swing_duration;  /**< 基于最近摆动相时长估算的下一摆动相预期时长     Estimate of how long the next swing will take based on the most recent swing times */
        
        // fsr箱参数
        float heel_fsr;                 /**< 经过校准的足跟压力传感器（FSR）读数    Calibrated FSR reading for the heel */
        float heel_fsr_upper_threshold; /**< 足跟压力传感器上限阈值     Upper threshold for the heel */
        float heel_fsr_lower_threshold; /**< 足跟压力传感器下限阈值     Lower threshold for the heel */
        float toe_fsr;                  /**< 经过校准的足趾压力传感器（FSR）读数    Calibrated FSR reading for the toe */
        float toe_fsr_upper_threshold;  /**< 足趾压力传感器上限阈值     Upper threshold for the toe */
        float toe_fsr_lower_threshold;  /**< 足趾压力传感器下限阈值     Lower threshold for the toe */
        
        bool ground_strike;             /**< 从摆动相切换至任意FSR检测到触地时触发  Trigger when we go from swing to one FSR making contact. */
        bool toe_strike;                /**< 检测到脚尖着地后触发，此发生在上次检测到脚尖离地之后   Trigger when we detect toe strike after the last detcted toe off */
        bool toe_off;                   /**< 当我们从脚尖FSR接触地面变为摆动状态时触发  Trigger when we go from toe FSR making contact to swing. */
        bool toe_on;                    /**< 当我们从脚尖FSR未接触地面变为接触地面时触发    Trigger when we go from toe FSR not making contact to making contact */
        bool heel_stance;               /**< 当脚跟FSR与地面接触时为高电平  High when the heel FSR is in ground contact */
        bool toe_stance;                /**< 当脚尖FSR与地面接触时为高电平  High when the toe FSR is in ground contact */
        bool prev_heel_stance;          /**< 上一次控制周期中脚跟FSR与地面接触时为高电平, 用与和上一个控制周期作对比    High when the heel FSR was in ground contact on the previous iteration */
        bool prev_toe_stance;           /**< 上一次控制周期中脚趾FSR与地面接触时为高电平, 用与和上一个控制周期作对比    High when the toe FSR was in ground contact on the previous iteration */
        
        bool is_left;                               /**< 1 if the side is on the left, 0 otherwise */
        bool is_used;                               /**< 1 if the side is used, 0 otherwise */
        bool do_calibration_toe_fsr;                /**< 脚尖FSR校准是否应执行的标志    Flag for if the toe calibration should be done */
        bool do_calibration_refinement_toe_fsr;     /**< 脚尖FSR校准优化是否应执行的标志    Flag for if the toe calibration refinement should be done */
        bool do_calibration_heel_fsr;               /**< 脚跟FSR校准是否应执行的标志    Flag for if the heel calibration should be done */
        bool do_calibration_refinement_heel_fsr;    /**< 脚跟FSR校准优化是否应执行的标志    Flag for if the heel calibration refinement should be done */

        float ankle_angle_at_ground_strike;         /**< 触地时刻踝关节角度估算值   Estimated angle of the ankle when at ground strike */
        float expected_duration_window_upper_coeff; /**< 新一步触地判定时间窗口上限系数, 用于计算时间窗口上限的系数，该时间窗口用于判断脚着地是否视为新的步态周期    Factor to multiply by the expected duration to get the upper limit of the window to determine if a ground strike is considered a new step. */
        float expected_duration_window_lower_coeff; /**< 新一步触地判定时间窗口下限系数, 用于计算时间窗口下限的系数，该时间窗口用于判断脚着地是否视为新的步态周期    Factor to multiply by the expected duration to get the lower limit of the window to determine if a ground strike is considered a new step. */

        Inclination inclination;        /**< 用于存储倾斜度相关的数据   Data for inclination */
};

#endif
