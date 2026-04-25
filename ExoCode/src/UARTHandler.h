/**
    @file UARTHandler.h
    @author Chance Cuddeback
    @brief 单例类，用于管理 UART 数据。非线程安全。该类会对接收的消息进行排队。
    @date 2022-09-07
*/


#ifndef UARTHandler_h
#define UARTHandler_h

//#include "Board.h"
//#include "ParseIni.h"
//#include "ExoData.h"
//#include "Utilities.h"
#include "UART_msg_t.h"

#include "Arduino.h"
#include <stdint.h>

#define MAX_NUM_SIDES 2             //Seems unlikely there would be any more
#define MAX_NUM_JOINTS_PER_SIDE 2   //现有的PCB板仅支持每个侧边配置两个电机, 如果设计了新的PCB, 请更新这个变量  Current PCB can only do 2 motors per side, if you have made a new PCB, update.
#define MAX_RAW_BUFFER_SIZE 256     // 原始缓冲区最大大小
#define MAX_DATA_SIZE 32            // 数据域最大长度
#define UART_DATA_TYPE short int //串口传输时用短整型存数据；改类型时，要同步改 pack_float/unpack_float 代码    If type is changes you will need to comment/uncomment lines in pack_float and unpack_float
#define FIXED_POINT_FACTOR 100      //浮点数 ↔ 定点数转换系数
#define UART_BAUD 256000

#define MAX_RX_LEN 64       //单次接收数据包最大 64 字节    Bytes
#define RX_TIMEOUT_US 1000  //接收超时：1000 微秒（1ms），超时就不再等数据  Microseconds

/* SLIP special character codes */
#define END             0300    /* 表示数据包结束, 发送方：发完一整包数据，最后发一个 END, 接收方：看到 END → 判定一帧数据接收完成  Indicates end of packet */
#define ESC             0333    /* 转义前缀（告诉接收方：下一个字节是数据，不是控制符）   Indicates byte stuffing */
#define ESC_END         0334    /* 转义表示：数据里真实的 END 字节  ESC ESC_END means END data byte */
#define ESC_ESC         0335    /* 转义表示：数据里真实的 ESC 字节  ESC ESC_ESC means ESC data byte */

#if defined(ARDUINO_TEENSY36) || defined(ARDUINO_TEENSY41)
#define MY_SERIAL Serial8       //teensy串口
#elif defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
#define MY_SERIAL Serial1       //Nano串口
#else 
#error No Serial Object Found
#endif

/**
 * @brief Singleton Class to handle the UART Work. 
 * 
 */
class UARTHandler
{
    public:
        /**
         * @brief 单例模式  获取实例对象  Get the instance object
         * 
         * @return 单例对象的指针   UARTHandler* A reference to the singleton
         */
        static UARTHandler* get_instance();

        /**
         * @brief 打包并发送一条 UART 消息
         * 
         * @param msg_id    消息ID，用于接收端识别、对应数据, 区分不同的消息类型, 消息类型定义在 uart_commands.h 中
         * @param len       数据长度
         * @param joint_id  与该数据关联的关节ID
         * @param buffer    数据载荷（要发送的有效数据）
         */
        void UART_msg(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *buffer);
        void UART_msg(UART_msg_t msg);

        /**
         * @brief 检查是否有接收数据。若有数据则读取消息，若等待超时则停止接收。核心轮询函数，负责检查、
         *        接收和解析来自UART接口的数据，并将其转换为结构化的消息(UART_msg_t)
         * 
         * @param timeout_us 超时时间（单位：微秒）
         * @return 解析完成的 UART 消息结构体, 结构体定义在 UART_msg_t.h 中
         */
        UART_msg_t poll(float timeout_us = RX_TIMEOUT_US);

        /**
         * @brief 检查UART接收缓冲区中是否有可用数据
         * 
         * @return uint8_t UART接收缓冲区中可用的字节数（Arduino平台最大为64字节）
         */
        uint8_t check_for_data();

    private:
        /**
         * @brief Construct a new UARTHandler object
         * 
         */
        UARTHandler();

        void _pack(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data, uint8_t *data_to_pack);

        UART_msg_t _unpack(uint8_t* data, uint8_t len);

        uint8_t _get_packed_length(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data);

        void _send_packet(uint8_t* p, uint8_t len);

        int _recv_packet(uint8_t *p, uint8_t len = MAX_RX_LEN);

        void _send_char(uint8_t val);

        uint8_t _recv_char(void);

        uint8_t _time_left(uint8_t should_latch = 0);

        //将_partial_packet_len重置为零
        void _reset_partial_packet();

        /* Data */
        //circular_buffer<uint8_t, 64> _rx_raw;

        float _timeout_us = RX_TIMEOUT_US;
        
        /*
        刚开机没有收到任何数据，有效长度本来就是 0；
        不初始化会是随机垃圾值，导致缓冲区越界、崩溃；
        接收数据时要从 0 开始累加计数，才能算对真实长度。
        */
        uint8_t _partial_packet[MAX_RX_LEN];
        uint8_t _partial_packet_len = 0;
        uint8_t _msg_buffer[MAX_RX_LEN];
        uint8_t _msg_buffer_len = 0;

};

#endif