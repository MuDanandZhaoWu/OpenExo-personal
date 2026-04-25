/**
 * @file ExoData.h
 *
 * @brief 声明一个类，该类用于存储数据，供外骨骼（Exo）访问使用  ExoData.h 就是被抽出来的 “纯数据容器”。      Declares a class used to store data for the Exo to access 
 * 
 * @author P. Stegall 
 * @date Jan. 2022
*/


#ifndef ExoData_h
#define ExoData_h

#include "Arduino.h"

#include "SideData.h"
#include <stdint.h>
#include "ParseIni.h"
#include "Board.h"
#include "StatusLed.h"
#include "StatusDefs.h"
#include "Config.h"
#include "Utilities.h"

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
	#if BATTERY_SENSOR == 260
		#include <Adafruit_INA260.h>
	#elif BATTERY_SENSOR == 219
		#include <Adafruit_INA219.h>
	#endif
#endif

/* 
 * ExoData was broken out from the Exo class to have it mirrored on a second microcontroller that handles BLE.
 * It doesn't need to be done this way if we aren't, and is pretty cumbersome.
 * Just thought you might be wondering about the approach.
 */
/* 
 * 译 
 * ExoData 是从 Exo 类中拆分出来的，目的是让它能在负责处理 BLE（蓝牙低功耗）的第二个微控制器上进行数据镜像。
 * 如果我们不采用双MCU架构，就没必要这么设计，而且这种写法本身比较繁琐。
 * 只是觉得你可能会疑惑为什么要这么实现，所以备注一下。
 */

//Note: Status values are in StatusDefs.h       状态值定义在StatusDefs.h头文件

// 该类型用于遍历每个关节的方法，要求函数以 JointData 作为输入参数，并且无返回值。Type used for the for each joint method, the function should take JointData as input and return void
typedef void (*for_each_joint_function_t) (JointData*, float*); 

/**
 * @brief Class to store all the data related to the exo
 */
class ExoData 
{
	public:
        ExoData(uint8_t* config_to_send); // 构造函数   Constructor
        
        /**
         * @brief Reconfigures the the exo data if the configuration changes after constructor called.
         * 
         * @param configuration array
         */
        void reconfigure(uint8_t* config_to_send);
        
        /**
         * @brief 对每个关节执行指定函数, 传达一些需要所用关节同时响应的指令
         * 
         * @param 指向要对每个已使用关节执行的函数的指针
         */
        template <typename F>
        void for_each_joint(F &&func)
        {
                func(&left_side.hip, NULL);
                func(&left_side.knee, NULL);
                func(&left_side.ankle, NULL);
                func(&left_side.elbow, NULL);
                func(&left_side.arm_1, NULL);
                func(&left_side.arm_2, NULL);
                func(&right_side.hip, NULL);
                func(&right_side.knee, NULL);
                func(&right_side.ankle, NULL);
                func(&right_side.elbow, NULL);
                func(&right_side.arm_1, NULL);
                func(&right_side.arm_2, NULL);
        }
        template <typename F>
        void for_each_joint(F &&func, float* args)
        {
                func(&left_side.hip, args);
                func(&left_side.knee, args);
                func(&left_side.ankle, args);
                func(&left_side.elbow, args);
                func(&left_side.arm_1, args);
                func(&left_side.arm_2, args);
                func(&right_side.hip, args);
                func(&right_side.knee, args);
                func(&right_side.ankle, args);
                func(&right_side.elbow, args);
                func(&right_side.arm_1, args);
                func(&right_side.arm_2, args);
        }

        //Returns a list of all of the joint IDs that are currently being used
        uint8_t get_used_joints(uint8_t* used_joints);

        /**
         * @brief 根据关节ID获取对应的关节数据指针, 按「关节数字ID」查关节的工具函数
         * 
         * @param id 关节ID
         * @return JointData* 指向该ID对应关节的JointData类对象的指针
         */
        JointData* get_joint_with(uint8_t id);
        
        /**
         * @brief 打印所用外骨骼数据    Prints all the exo data
         */
        void print();

        /**
         * @brief Set the status object  设置系统状态
         * 
         * @param status_to_set status_defs::messages::status_t
         */
        void set_status(uint16_t status_to_set);
        
        /**
         * @brief Get the status object
         * 
         * @return uint16_t status_defs::messages::status_t
         */
        uint16_t get_status(void);

        /**
         * @brief 这是为所有在使用的的关节执行设置默认参数    Set the default controller parameters for the current controller. These are the first row in the controller csv file on the SD Card
         *
         */
        void set_default_parameters();
        
        /**
         * @brief 为指定控制器设置默认控制参数。这些参数取自SD卡上控制器CSV文件的第一行       Set the default controller parameters for the current controller. These are the first row in the controller csv file on the SD Card
         * 
         */
        void set_default_parameters(uint8_t id);

        /**
         * @brief 启动试验前校准流程    Start the pretrial calibration process
         * 
         */
        void start_pretrial_cal();
		
		#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
			#if BATTERY_SENSOR == 260
				Adafruit_INA260 ina260 = Adafruit_INA260();
			#elif BATTERY_SENSOR == 219
				Adafruit_INA219 ina219;
			#endif
		#endif
		
		/**
         * @brief Communicate with the power sensor and pull battery-related information such as voltage, current and power.
         * 
         */
		float get_batt_info(uint8_t batt_info_type);
        
        bool sync_led_state;    /**< 同步LED的状态变量，用于指示系统内部或与其他设备的同步状态  State of the sync led */
        bool estop;             /**< 紧急停止(e-stop)状态标志   State of the estop */
        float battery_value;    /**< Could be Voltage or SOC, depending on the battery type*/
	float filtered_batt_pwr = 0;    /**< Filtered battery power*/
        SideData left_side;     /**< Data for the left side */
        SideData right_side;    /**< Data for the right side */

        uint32_t mark;          /**< 时间标记变量，用于定时功能，目前主要用于nano控制器的时间记录       Used for timing, currently only used by the nano */

        uint8_t* config;        /**< 指向配置数组的指针，存储外骨骼系统的配置信息       Pointer to the configuration array */
        uint8_t config_len;     /**< Length of the configuration array */

        int error_code;         /**< 系统当前的错误代码，用于标识发生的故障类型 Current error code for the system */
        int error_joint_id;
        bool user_paused;       /**< 用户暂停标志，指示用户是否主动暂停了系统运行       If the user has paused the system */

        int hip_torque_flag = 0;    /**< 髋关节扭矩传感器使用标志       Flag to determine if we want to use torque sensor for that joint */
        int knee_torque_flag = 0;   /**< Flag to determine if we want to use torque sensor for that joint */
        int ankle_torque_flag = 0;  /**< Flag to determine if we want to use torque sensor for that joint */
        int elbow_torque_flag = 0;  /**< Flag to determine if we want to use torque sensor for that joint */
        int arm_1_torque_flag = 0;  /**< Flag to determine if we want to use torque sensor for that joint */
        int arm_2_torque_flag = 0;  /**< Flag to determine if we want to use torque sensor for that joint */
		
        private:
        uint16_t _status;           /**< Status of the system*/
};

#endif
