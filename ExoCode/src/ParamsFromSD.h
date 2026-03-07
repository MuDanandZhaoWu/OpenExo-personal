/**
 * @file ParamsFromSD.h
 *
 * @brief 声明用于从 SD 卡读取控制器参数的函数，并定义参数文件的映射关系 Declares the functions to pull controller parameters from the SD card and defines the mapping to the parameter files.
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/

#ifndef ParamsFromSD_h
#define ParamsFromSD_h

#include "ExoData.h"
#include "ParseIni.h"
#include "Utilities.h"

#include <SD.h>
#include <SPI.h>
#include <map>
#include <string>

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
    #ifndef SD_SELECT
        #define SD_SELECT BUILTIN_SDCARD
    #endif
    
    typedef std::map<uint8_t, std::string> ParamFilenameKey;
    
    /**
     * @brief Types of errors when reading the SD card
     */
    namespace param_error
    {
        const uint8_t num_joint_ids = 5;                            /**< Number of bits the joint type ids need 关节类型id 所需的位数 */
        const uint8_t SD_not_found_idx = num_joint_ids;             /**< Error when SD card isn't present */
        const uint8_t file_not_found_idx = SD_not_found_idx + 1;    /**< Error when file is not found on the SD card */
    }
    
    /**
     * @brief 用于建立控制器与文件路径映射关系的命名空间，建立电机控制器与文件路径的映射关系    Namespace with map to between controller and file location
     */
    namespace controller_parameter_filenames
    {
        const ParamFilenameKey hip
        {
            {(uint8_t)config_defs::hip_controllers::disabled,"hipControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::hip_controllers::zero_torque,"hipControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::hip_controllers::franks_collins_hip, "hipControllers/franksCollinsHip.csv"},
            {(uint8_t)config_defs::hip_controllers::spline, "hipControllers/spline.csv"},
            {(uint8_t)config_defs::hip_controllers::constant_torque, "hipControllers/constantTorque.csv"},
            {(uint8_t)config_defs::hip_controllers::chirp,"hipControllers/chirp.csv"},
            {(uint8_t)config_defs::hip_controllers::step,"hipControllers/step.csv"},
            {(uint8_t)config_defs::hip_controllers::phmc,"hipControllers/PHMC.csv"},
        };
        
        const ParamFilenameKey knee
        {
            {(uint8_t)config_defs::knee_controllers::disabled,"kneeControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::knee_controllers::zero_torque,"kneeControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::knee_controllers::constant_torque, "kneeControllers/constantTorque.csv"},
            {(uint8_t)config_defs::knee_controllers::chirp,"kneeControllers/chirp.csv"},
            {(uint8_t)config_defs::knee_controllers::step,"kneeControllers/step.csv"},
        };
        
        const ParamFilenameKey ankle
        {
            {(uint8_t)config_defs::ankle_controllers::disabled,"ankleControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::ankle_controllers::zero_torque,"ankleControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::ankle_controllers::pjmc,"ankleControllers/PJMC.csv"},
            {(uint8_t)config_defs::ankle_controllers::zhang_collins,"ankleControllers/zhangCollins.csv"},
            {(uint8_t)config_defs::ankle_controllers::spline,"ankleControllers/spline.csv"},
            {(uint8_t)config_defs::ankle_controllers::constant_torque, "ankleControllers/constantTorque.csv"},
            {(uint8_t)config_defs::ankle_controllers::trec,"ankleControllers/trec.csv"},
            {(uint8_t)config_defs::ankle_controllers::chirp,"ankleControllers/chirp.csv"},
            {(uint8_t)config_defs::ankle_controllers::step,"ankleControllers/step.csv"},
			{(uint8_t)config_defs::ankle_controllers::spv2,"ankleControllers/spv2.csv"},
			{(uint8_t)config_defs::ankle_controllers::pjmc_plus,"ankleControllers/pjmc_plus.csv"},
        };

        const ParamFilenameKey elbow
        {
            {(uint8_t)config_defs::elbow_controllers::disabled,"elbowControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::elbow_controllers::zero_torque,"elbowControllers/zeroTorque.csv"},
            {(uint8_t)config_defs::elbow_controllers::elbow_min_max, "elbowControllers/elbowMinMax.csv"},
            {(uint8_t)config_defs::elbow_controllers::chirp,"elbowControllers/chirp.csv"},
            {(uint8_t)config_defs::elbow_controllers::step,"elbowControllers/step.csv"},
        };

        const ParamFilenameKey arm_1
        {
            {(uint8_t)config_defs::arm_1_controllers::disabled,"arm1Controllers/zeroTorque.csv"},
            {(uint8_t)config_defs::arm_1_controllers::zero_torque,"arm1Controllers/zeroTorque.csv"},
            {(uint8_t)config_defs::arm_1_controllers::constant_torque,"arm1Controllers/constantTorque.csv"},
            {(uint8_t)config_defs::arm_1_controllers::spline,"arm1Controllers/spline.csv"},
        };

        const ParamFilenameKey arm_2
        {
            {(uint8_t)config_defs::arm_2_controllers::disabled,"arm2Controllers/zeroTorque.csv"},
            {(uint8_t)config_defs::arm_2_controllers::zero_torque,"arm2Controllers/zeroTorque.csv"},
            {(uint8_t)config_defs::arm_2_controllers::constant_torque,"arm2Controllers/constantTorque.csv"},
            {(uint8_t)config_defs::arm_2_controllers::spline,"arm2Controllers/spline.csv"},
        };

    };
    
    /**
     * @brief 打印SD卡读取时的错误信息  Prints name of error message
     *
     * @param error identifier
     */
    void print_param_error_message(uint8_t error_type);
    
    /**
     * @brief 从SD卡读取文件并将其设置到exo_data对象中对应的控制器参数
     * 有关输入参数的详细说明请参考ParseIni
     * 
     * @param joint_id 关节ID 
     * @param controller_id 控制器ID 
     * @param set_num 要从SD卡读取的参数集编号
     * @param exo_data 数据存储的目标位置 
     * 
     * @return 错误码（整数类型）
     */
    uint8_t set_controller_params(uint8_t joint_id, uint8_t controller_id, uint8_t set_num, ExoData* exo_data);

#endif
#endif
