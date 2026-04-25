#ifndef UARTMSG_H
#define UARTMSG_H

#define UART_MSG_T_MAX_DATA_LEN 128
#include "Arduino.h"
#include "Logger.h"


/*
结构体定义了在外骨骼系统中通过UART串口传输的数据包格式，
用于Teensy主控制器与其他模块（如Nano）之间的通信
*/
typedef struct
{
  uint8_t command;      //指令类型标识符，用于确定消息的用途或类型, command定义在uart_commands.h中
  uint8_t joint_id;
  float data[UART_MSG_T_MAX_DATA_LEN];  //数组长度为最大发送消息长度
  uint8_t len;      //数据长度，表示实际使用的data数组元素个数
} UART_msg_t;

namespace UART_msg_t_utils
{
    static void print_msg(UART_msg_t msg)
    {
        logger::println("UART_command_utils::print_msg->Msg: ");
        logger::print(msg.command); logger::print("\t");
        logger::print(msg.joint_id); logger::print("\t");
        logger::print(msg.len); logger::println();
        for (int i=0; i<msg.len; i++)
        {
           logger::print(msg.data[i]); logger::print(", ");
        }
        logger::println();
    }
};


#endif