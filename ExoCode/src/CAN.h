/**
 * @file CAN.h
 * @author Chancelor Cuddeback
 * @brief Uses the FlexCan library to send and receive CAN messages.
 * @date 2023-07-18
 * 
 */

#ifndef CAN_H
#define CAN_H

#include "Logger.h"
#include "Arduino.h"

 //Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

#include "FlexCAN_T4.h"
#if defined(ARDUINO_TEENSY36)
    static FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#elif defined(ARDUINO_TEENSY41)
    static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
#endif

/**
 * @brief CAN class for sending and receiving CAN messages. Singleton
 * 
 */
class CAN 
{
    public:
        /**
         * @brief Get the Singleton object 单例模式，防止出现多个实例同时操作一个硬件（如teensy）导致收发冲突
         * 
         * @return CAN* 
         */
        static CAN* getInstance()
        {
            static CAN* instance = new CAN;
            return instance;
        }

        /**
         * @brief Send a CAN message
         * 
         * @param msg CAN_message_t to send
         */
        bool send(CAN_message_t msg)
        {
            if(!Can0.write(msg))        //判断报文是否成功写入缓冲区,写入失败进入if语句发送失败信息打印失败报文的信息
            {
                logger::println("Error Sending" + String(msg.id), LogLevel::Error);
                return false;
            }
            return true;
        }

        /**
         * @brief Read a CAN message
         * 
         * @param msg destination for a received message
         * @return true only when a message was actually received
         */
        bool read(CAN_message_t& msg)
        {
            msg = CAN_message_t();
            return Can0.read(msg);
        }

        /**
         * @brief Read the next feedback frame for one motor without discarding
         * frames that belong to other motors on the shared CAN bus.
         */
        /**
         * @brief 读取指定电机的下一条反馈报文，共享CAN总线上其余电机的报文不会被丢弃
         *
         * @param motor_id 目标电机ID
         * @param extended true表示按扩展帧格式匹配电机ID，false表示按标准帧格式匹配
         * @param msg 找到目标电机反馈时，用于保存最新的一条CAN报文
         * @param received_us 找到目标电机反馈时，用于保存该报文的接收时间戳，单位为微秒
         * @return true 找到了目标电机的反馈报文
         * @return false 没有找到目标电机的反馈报文
         */
        bool read_for_motor(uint8_t motor_id, bool extended, CAN_message_t& msg,
                            uint32_t& received_us)
        {
            // 标记本次读取是否已经找到目标电机的反馈帧。
            bool found = false;

            // 先检查之前缓存下来的报文。缓存中保存的是其它电机读取时
            // 从共享CAN FIFO中顺手读到、但当时不属于目标电机的报文。
            for (uint8_t i = 0; i < pending_capacity; ++i)
            {
                //判断缓存里有没有目标电机报文
                if (_pending_used[i] &&
                    _matches_motor(_pending_messages[i], motor_id, extended))
                {
                    if (!found || static_cast<int32_t>(
                            _pending_received_us[i] - received_us) > 0) //对比时间戳，取时间戳最大的报文
                    {
                        msg = _pending_messages[i];
                        received_us = _pending_received_us[i];  //更新时间戳
                        found = true;
                    }

                    // 这条缓存已经被当前电机消费，后续不再保留(缓冲数组中对应使用过的报文丢弃)
                    _pending_used[i] = false;
                }
            }

            // Drain a bounded portion of the shared RX FIFO. Frames for other
            // motors are retained instead of being counted as this motor's
            // communication failure.
            // 再从硬件CAN接收FIFO中最多读取max_drain_per_read条，避免一次调用
            // 占用太久，同时尽量把目标电机的最新反馈读出来。
            for (uint8_t i = 0; i < max_drain_per_read; ++i)
            {
                CAN_message_t candidate;
                if (!Can0.read(candidate))
                {
                    // FIFO为空时停止读取。
                    break;
                }

                const uint32_t candidate_received_us = micros();
                if (_matches_motor(candidate, motor_id, extended))
                {
                    // 目标电机的反馈直接作为候选结果，同样保留最新的一条。
                    //如果此次读取到的就是目标电机对应的报文就直接将candidate回传给msg并且不在缓存中更新
                    if (!found || static_cast<int32_t>(
                            candidate_received_us - received_us) > 0)
                    {
                        msg = candidate;
                        received_us = candidate_received_us;
                        found = true;
                    }
                    continue;
                }

                // 不是目标电机的报文不能丢弃，缓存起来给对应电机之后读取。
                _cache_message(candidate, candidate_received_us);
            }
            // 返回是否找到了目标电机的反馈报文
            return found;
        }

        /**
         * @brief Remove already queued feedback for one motor before issuing a
         * fresh enable request. Other motors' frames remain available.
         */
        /**
         * @brief 在下发全新的电机使能指令前，清除缓存中属于该电机的所有排队反馈报文；避免上电 / 重启后读到过期位置、速度数据引发控制抖动、冲击
         * 其余电机的报文会完整保留，仍可正常读取使用
         */
        void discard_for_motor(uint8_t motor_id, bool extended)
        {
            for (uint8_t i = 0; i < pending_capacity; ++i)
            {
                if (_pending_used[i] &&
                    _matches_motor(_pending_messages[i], motor_id, extended))
                {
                    _pending_used[i] = false;
                }
            }

            for (uint16_t i = 0; i < max_drain_before_enable; ++i)
            {
                CAN_message_t candidate;
                // 读取共享CAN FIFO中的报文，直到FIFO为空或者达到最大读取次数。
                if (!Can0.read(candidate))
                {
                    break;
                }
                const uint32_t candidate_received_us = micros();
                // 如果读取到的报文不是目标电机的反馈，就缓存起来。
                if (!_matches_motor(candidate, motor_id, extended))
                {
                    _cache_message(candidate, candidate_received_us);
                }
            }
        }

    private:
        static const uint8_t pending_capacity = 16;
        static const uint8_t max_drain_per_read = 32;
        static const uint16_t max_drain_before_enable = 256;
        CAN_message_t _pending_messages[pending_capacity];
        uint32_t _pending_received_us[pending_capacity];
        bool _pending_used[pending_capacity];
        uint8_t _pending_write_index;

        //用于判断一条 CAN 报文 msg 是不是指定电机 motor_id 的反馈报文
        bool _matches_motor(const CAN_message_t& msg, uint8_t motor_id,
                            bool extended) const
        {
            //针对是否为扩展帧进行不同的判断
            if (extended)
            {
                // msg.len >= 6，说明数据长度至少 6 字节，是有效反馈帧(即至少接收到电机的驱动器ID号,电机位置等的数据)的条件之一
                return msg.flags.extended && msg.len >= 6 &&
                    ((msg.id & 0xFF) == motor_id);
            }
            // 标准帧的情况下, 电机指定数据帧的数据段的第一个字节为电机ID，且数据长度大于等于6
            return !msg.flags.extended && msg.len >= 6 &&
                (msg.buf[0] == motor_id);
        }

        //将一条非当前目标电机的有效 CAN 反馈报文存入软件环形缓存池 _pending_messages，实现同电机自动覆盖旧报文
        void _cache_message(const CAN_message_t& msg, uint32_t received_us)
        {
            if (msg.len == 0)
            {
                return;
            }
            for (uint8_t i = 0; i < pending_capacity; ++i)
            {
                if (_pending_used[i] &&
                    _same_feedback_source(_pending_messages[i], msg))
                {
                    _pending_messages[i] = msg;
                    _pending_received_us[i] = received_us;
                    return;
                }
            }
            for (uint8_t i = 0; i < pending_capacity; ++i)
            {
                if (!_pending_used[i])
                {
                    _pending_messages[i] = msg;
                    _pending_received_us[i] = received_us;
                    _pending_used[i] = true;
                    return;
                }
            }

            // If producers outrun consumers, replace the oldest ring slot;
            // the timestamp is retained so delayed frames cannot become fresh.
            _pending_messages[_pending_write_index] = msg;
            _pending_received_us[_pending_write_index] = received_us;
            _pending_used[_pending_write_index] = true;
            _pending_write_index =
                (uint8_t)((_pending_write_index + 1) % pending_capacity);
        }

        /*
        判断两条 CAN 反馈报文是否来自同一个电机驱动器（同一反馈源），用来识别缓存池内是否已
        经存在同一台电机的旧反馈报文，实现「同电机只保留最新一条反馈、自动覆盖旧报文」的缓存策略
        */
        bool _same_feedback_source(const CAN_message_t& lhs,
                                   const CAN_message_t& rhs) const
        {
            //两帧帧类型不一样或任意报文数据长度小于 1直接返回 false
            if (lhs.flags.extended != rhs.flags.extended ||
                lhs.len < 1 || rhs.len < 1)
            {
                return false;
            }
            //扩展帧判断同源
            if (lhs.flags.extended)
            {
                return (lhs.id & 0xFF) == (rhs.id & 0xFF);
            }
            return lhs.buf[0] == rhs.buf[0];
        }

        /**
         * @brief Construct a new CAN object and initialize the CAN bus. This is private 
         * because this is a singleton.
         * 初始化CAN总线，构造函数为私有函数，防止出现多个实例同时操作一个硬件（如teensy）导致收发冲突
         */
        CAN()
        {   
            _pending_write_index = 0;
            for (uint8_t i = 0; i < pending_capacity; ++i)
            {
                _pending_used[i] = false;
                _pending_received_us[i] = 0;
            }
            Can0.begin();
            Can0.setBaudRate(1000000);
        }
};

#endif
#endif
