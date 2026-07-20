/** 
 * @file Utilities.h
 *
 * @brief  
 *
 * @author P. Stegall & C. Cuddeback
 *
 * @date Jan. 2022
 */

#ifndef Utilities_h
#define Utilities_h

#include "ParseIni.h"
#include "Arduino.h"
#include <stdint.h>
#include <utility>  //std::pair
#include <queue>    //std::queue

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
#include "Board.h"
#include <map>
#include <Servo.h>
#endif

/**
 * @brief contains general utility functions for the exo
 */
namespace utils
{
    /**
     * @brief Takes in the joint id and returns if the left indicator bit is set as a bool
     *
     * @param joint id
     *
     * @return 1 if the id is for the left side
     */
    bool get_is_left(config_defs::joint_id id);
    
    /**
     * @brief Takes in the joint id and returns if the left indicator bit is set as a bool
     *
     * @param joint id
     *
     * @return 1 if the id is for the left side
     */
    bool get_is_left(uint8_t id);

    /**
     * @brief Takes in the joint id and returns the id with the left/right bits masked out.
     * Returning uint8_t rather than joint_id type since we have to typecast to do logical stuff anyways.
     *
     * @param joint id
     *
     * @return joint id with the side bits masked out
     */
    uint8_t get_joint_type(config_defs::joint_id id);
    
    /**
     * @brief Takes in the joint id and returns the id with the left/right bits masked out.
     * Returning uint8_t rather than joint_id type since we have to typecast to do logical stuff anyways.
     *
     * @param joint id
     *
     * @return joint id with the side bits masked out
     */
    uint8_t get_joint_type(uint8_t id);
    
    /**
     * @brief Returns the new state of the system based on the value and past state.
     * Takes in the current value, if the state is currently high, and the lower and upper threshold
     * Floats are used so that ints will be promoted but it will still work with floats.  May cause issues with very large ints.
     * Templates could be used in the future but seemed to have issues if all types were not present in the template, e.g. is_high is always a bool.
     * A schmitt trigger is a way of tracking if a noisy signal is high or low 
     * When it is low it must go above the upper threshold before it is high
     * When it is high it must go below the lower threshold before it is low.
     * This way if the signal crosses one threshold multiple times it won't register as changing multiple times.
     * 
     * @param The current signal reading
     * @param The current state of the trigger
     * @param The lower threshold a high signal must pass to go low
     * @param The upper threshold a low signal must pass to go high 
     * 
     * @return the state of the trigger.
     */
    /**
     * @brief 根据当前采样值与历史输出状态，返回施密特触发器最新输出状态
     * 入参支持浮点，传入整型会自动做类型提升，超大整型数值使用时可能存在精度问题。
     * 本函数原本可改用模板实现，但因存在固定布尔型参数（is_high），多类型混用会出现兼容问题，故暂未使用模板。
     * 施密特触发器用于带噪声的信号电平判定，消除信号抖动误触发：
     * 触发器当前为低电平状态时，信号必须超过上限阈值，才会切换为高电平；
     * 触发器当前为高电平状态时，信号必须低于下限阈值，才会切换为低电平。
     * 该机制可避免信号在单阈值附近反复抖动时，状态频繁误翻转。
     * 
     * @param value 当前信号采样读数
     * @param is_high 触发器当前的输出电平状态
     * @param lower_threshold 高电平切为低电平所需低于的下限阈值
     * @param upper_threshold 低电平切为高电平所需超过的上限阈值
     * 
     * @return 触发器更新后的电平状态
     */
    bool schmitt_trigger(float value, bool is_high, float lower_threshold, float upper_threshold);
    
    /**
     * @brief Limits the rate at which a value can change.  
     * This is useful when you would like a variable to gradually come on.
     * 
     * @param current value the system is moving towards
     * @param the previous value the system was at
     * @param pointer to the last time the output value was set
     * @param rate the value can change per ms
     * 
     * @return the next value to the system can be at based on the rate limit.
     */
    int rate_limit(int setpoint, int last_value, int* last_time, int rate_per_ms);
    
    /**
     * @brief sets/clears the specified bit in a unit8_t.
     * 
     * @param The original uint8_t 
     * @param The bit value you would like to use
     * @param The location you are placing that bit.
     * 
     * @return The uint8_t with the bit updated
     */
    uint8_t update_bit(uint8_t original, bool val, uint8_t loc);
    
    /**
     * @brief sets/clears the specified bit in a unit16_t.
     * 
     * @param The original uint16_t 
     * @param The bit value you would like to use
     * @param The location you are placing that bit.
     * 
     * @return The uint16_t with the bit updated
     */
    uint16_t update_bit(uint16_t original, bool val, uint8_t loc);
    
    /**
     * @brief Returns the bit in a specific location in a uint8_t
     * 
     * @param the uint8_t to read
     * @param the location of the bit to read
     * 
     * @return the value at the bit location
     */
    bool get_bit(uint8_t original, uint8_t loc);
    
    /**
     * @brief Returns the bit in a specific location in a uint16
     * 
     * @param the uint16_t to read
     * @param the location of the bit to read
     * 
     * @return the value at the bit location
     */
    bool get_bit(uint16_t original, uint8_t loc);
    
    /**
     * @brief converts from degrees to radians
     *
     * @param a degree value
     *
     * @return the value converted to radians 
     */
    float degrees_to_radians(float);
    
    /**
     * @brief converts from radians to degrees
     *
     * @param a radian value
     *
     * @return the value converted to degrees 
     */
    float radians_to_degrees(float);

    /**
     * @brief Searches str for 'rmv characters and deletes them all, returns new string
     */
    String remove_all_chars(String str, char rmv);
    String remove_all_chars(char* arr, int len, char rmv);

    /**
     * @brief given and integer, return the number of characters in it
     */
    int get_char_length(int ofInt);
    
    /**
     * @brief Checks if all elements of the array are equal. Arrays must be the same length and type
     */
    template <typename T>
    int elements_are_equal(T arr1, T arr2, int length)
    {
        for (int i=0; i<length; i++)
        {
            if(arr1[i] != arr2[i])
            {
                return 0;
            }
        }
        return 1;
    };

    /**
     * @brief Sets arr2 elements equal to arr1 elements. Arrays must be the same length and type
     */
    template <typename T>
    void set_elements_equal(T arr1, T arr2, int length)
    {
        for (int i=0; i<length; i++)
        {
            arr1[i] = arr2[i];
        }
    };
    
    /**
     * @brief Class used to check the loop speed without serial prints, by toggling a pin after initialized, toggle will need to be called each loop
     */
    class SpeedCheck
    {
        public:
            SpeedCheck(int pin);
            
            /**
             * @brief toggles the pin attached to the object
             */
            void toggle();
            
        private:
            int _pin;       /**< Pin to use for the check */
            bool _state;    /**< State of the pin */
    };
    
    /**
     * @brief Returns 1 if system uses little endian floating points.  This confirms that the floating points match if not the byte order needs to be flipped.
     * Not tested with big endian or 64 bit systems
     *
     * @return 1 if system is little endian
     */
    bool is_little_endian();
    
    /**
     * @brief Takes in a float and a byte array reference
     * Puts the bytes of the float into the array in little endian 
     * Not tested with big endian or 64 bit systems
     * 
     * @param float to convert
     * @param array of sizeof(float) to store the individual bytes
     */
    void float_to_uint8(float num_to_convert, uint8_t *converted_bytes);
    
    /**
     * @brief Takes in a byte array address in little endian form containing a broken up float
     * Returns a reconstituted float from the bytes in the form (endianess) the system uses.
     * Not tested with big endian or 64 bit systems
     * 
     * @param array of size sizeof(float) containing the bytes to convert
     * @param pointer to place the converted value into
     */
    void uint8_to_float(uint8_t *bytes_to_convert, float *converted_float);
    
    /**
     * @brief converts float into individual bytes of a fixed point short.  
     * 
     * @param float to convert
     * @param array of sizeof(short int) to store the individual bytes
     */
    void float_to_short_fixed_point_bytes(float num_to_convert, uint8_t *converted_bytes, uint8_t factor);
    
    /**
     * @brief converts a set of bytes in a fixed point short int into a float
     * 
     * @param array of size sizeof(short int) containing the bytes to convert
     * @param pointer to place the converted value into
     */
    void short_fixed_point_bytes_to_float(uint8_t *bytes_to_convert, float *converted_val, uint8_t factor);
    
    /**
     * @brief Exponential Weighted Moving Average. Used to smooth out noisy data
     * 
     * @param new_value New value to add to the filter
     * @param filter_value Previous value of the filter
     * @param alpha Tuning parameter, 0.0 < alpha < 1.0
     * @return float 
     */
    float ewma(float new_value, float filter_value, float alpha);

    /**
     * @brief Never returns from this function, used for critical errors
     * 
     * @param message 
     */
    void spin_on_error_with(String message);

    /**
     * @brief Checks if two floats are close to each other within a tolerance
     * 
     * @param val1 
     * @param val2 
     * @param tolerance 
     * @return true 
     * @return false 
     */
    bool is_close_to(float val1, float val2, float tolerance);

    /**
     * @brief Given a set of data and maximum size. Returns the new mean and standard deviation
     * 
     * @param set Queue of data to calculate the mean and standard deviation of
     * @return std::pair<float, float> Mean and standard deviation, respectively
     */
    /**
     * @brief 输入一组数据，计算并返回该组数据的均值与标准差
     *
     * @param set 待计算均值、标准差的浮点数据队列
     * @return std::pair<float, float> 成对返回两个浮点数，依次为均值、标准差
     */
    std::pair<float, float> online_std_dev(std::queue<float> set);
    
    /**
     * @brief Checks if a value is outside of a range
     * 
     * @param val
     * @param min 
     * @param max 
     * @return true 
     * @return false 
     */
    bool is_outside_range(float val, float min, float max);
	
	/**
     * @brief Attach the servos
     * 
     */
	void init_servos();
	
	/**
     * @brief Actuate a servo
     * 
     * @param servo_pin
     * @param target_angle 
     */
	void actuate_servo(uint8_t servo_pin, uint8_t target_angle);
}

#endif