#include "UARTHandler.h"
#include "Utilities.h"
#include "Logger.h"

#define MAX_NUM_LEGS 2
#define MAX_NUM_JOINTS_PER_LEG 2 //Current PCB can only do 2 motors per side, if you have made a new PCB, update.
#define UART_DATA_TYPE short int //If type is changes you will need to comment/uncomment lines in pack_float and unpack_float
#define FIXED_POINT_FACTOR 100

// Nano -> Teensy sends raw floats; Teensy -> Nano stays fixed-point ints.
#if defined(ARDUINO_ARDUINO_NANO33BLE) || defined(ARDUINO_NANO_RP2040_CONNECT)
#define UART_PACK_FLOATS 1
#define UART_UNPACK_FLOATS 0
#else
#define UART_PACK_FLOATS 0
#define UART_UNPACK_FLOATS 1
#endif

//Set to 1 to enable debug prints
#define DEBUG_UART_HANDLER 0

//数据包索引
typedef enum 
{
  COMMAND = 0,      // 指令字段在第 0 个字节
  JOINT_ID = 1,     // 关节ID字段在第 1 个字节
  DATA_START = 2    // 真正的数据从第 2 个字节开始
} UARTPackingIndex;


UARTHandler::UARTHandler()
{
  //_rx_raw = CircularBuffer_<char>(_rx_raw_buffer, _k_bufferSize);
  MY_SERIAL.begin(UART_BAUD);
  MY_SERIAL.setTimeout(0);
}

//定义 UARTHandler 类的成员函数 get_instance，该函数返回一个指向 UARTHandler 对象的指针
UARTHandler* UARTHandler::get_instance()
{
    static UARTHandler* instance = new UARTHandler();
    return instance;
}

//UART_msg函数的底层实现函数
void UARTHandler::UART_msg(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *buffer)
{
    uint8_t _packed_len = _get_packed_length(msg_id, len, joint_id, buffer);

    #if DEBUG_UART_HANDLER
    logger::print("UARTHandler::UART_msg->Packing Bytes: "); logger::println(_packed_len);
    #endif

    uint8_t _byte_data[_packed_len] = {0};
    //将消息ID、长度、关节ID和浮点数缓冲区中的数据打包到 _byte_data 数组中
    _pack(msg_id, len, joint_id, buffer, _byte_data);

    #if DEBUG_UART_HANDLER
   logger::println("UARTHandler::UART_msg->Packed data:");
   for (int i=0; i<_packed_len; i++)
   {
     logger::print(_byte_data[i]); logger::print(", ");
   }
   logger::println();
    #endif

    _send_packet(_byte_data, _packed_len);
    //阻塞等待，直到串口发送缓冲区里的所有字节都被硬件真正发走，然后才继续往下执行
    MY_SERIAL.flush();

    #if DEBUG_UART_HANDLER
   logger::println("UARTHandler::UART_msg->Flushed tx buffer");
    #endif
}

//UART_msg的上层封装, 从结构体 msg 里取出 command、len、joint_id、data，直接调用第一个UART_msg函数
void UARTHandler::UART_msg(UART_msg_t msg)
{
    #if DEBUG_UART_HANDLER
        logger::print("UARTHandler::UART_msg->Sending Message");
        UART_msg_t_utils::print_msg(msg);
    #endif

    UART_msg(msg.command, msg.len, msg.joint_id, msg.data);
}

/**
 * @brief 检查并接收串口数据
 * 
 * 该函数会检查是否有可用的串口数据，如果有则接收并解析成消息结构。
 * 支持处理分段数据包和超时机制。
 * 
 * @param timeout_us 超时时间（微秒）
 * @return UART_msg_t 返回解析后的消息结构，如果没有数据则返回空消息
 */
UART_msg_t UARTHandler::poll(float timeout_us)
{
    //创建一个空消息作为没有数据时的返回值
    static UART_msg_t empty_msg = {0, 0, 0, 0};
    _timeout_us = timeout_us;
    
    //调用 check_for_data() 检查是否有可用数据
    uint32_t _available_bytes = check_for_data();
    if (!_available_bytes) {return empty_msg;}

    #if DEBUG_UART_HANDLER
        logger::print("UARTHandler::poll->Bytes Available: "); logger::println(_available_bytes);
    #endif

    //创建接收缓冲区, 用于存储数据
    uint8_t _msg_buffer[MAX_RX_LEN];
    int _recv_len = _recv_packet(_msg_buffer, MAX_RX_LEN);
    
    //如果_recv_len > 0，说明成功接收了数据
    if (_recv_len > 0)
    {
      //处理可能存在的部分数据包与当前接收的数据进行合并
      if (_partial_packet_len) 
      {
        //This occurs if there was a timeout during _recv_packet and we have a complete message
        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::poll->_recv_len > 0 && _partial_packet_len");
        #endif

        //Shift _msg_buffer _partial_packet_len bytes to fit the previous partial packet using memmove
        //把接收的 _msg_buffer 数据向后移动 _partial_packet_len 字节，避免覆盖；
        memmove(_msg_buffer + _partial_packet_len, _msg_buffer, _recv_len);
        
        //Copy the partial packet to the beginning of the buffer
        //把缓存的 _partial_packet复制到 _msg_buffer 开头, 与memmove配合形成先做数据地址后移再做新数据填充的操作,避免数据被覆盖
        memcpy(_msg_buffer, _partial_packet, _partial_packet_len);

        _recv_len += _partial_packet_len;

        //清空部分数据包缓存
       _reset_partial_packet();
     }

      //把合并后的完整字节缓冲区，解析为包含「指令、关节 ID、数据」的 UART_msg_t 结构化消息
      UART_msg_t msg = _unpack(_msg_buffer, _recv_len);

      #if DEBUG_UART_HANDLER
          logger::print("UARTHandler::poll->Got Message: ");
          UART_msg_t_utils::print_msg(msg);
      #endif

      //返回解析后的消息结构体
      return msg;
    }

    if (_recv_len < 0)
    {
      //This only occurs if there was a timeout during the previous _recv_packet and an end flag before any new data, in this case the partial packet is the full message
      /* 
      _recv_len < 0这种情况仅在以下条件同时满足时发生：上一次调用 _recv_packet 接收数据时触发了超时，
      且在接收到任何新数据之前检测到了「数据包结束标记（end flag）」；
      此时缓存的「部分数据包（partial packet）」实际上就是完整的消息。
      */
      #if DEBUG_UART_HANDLER
        logger::println("UARTHandler::poll->_recv_len < 0");
      #endif

      //Append the _partial packet to the full message
      //把缓存的部分数据包复制到接收缓冲区
      memcpy(_msg_buffer, _partial_packet, _partial_packet_len);
       
      #if DEBUG_UART_HANDLER
        logger::println("UARTHandler::poll->_msg_buffer after copyting _packed_data: ");
          for (int i=0; i<(_partial_packet_len); i++)
          {
            logger::print(_msg_buffer[i]); logger::print(", ");
          }
          logger::println();
      #endif

      //解析缓冲区数据
      UART_msg_t msg = _unpack(_msg_buffer, _partial_packet_len);

      _reset_partial_packet();

      #if DEBUG_UART_HANDLER
          logger::print("UARTHandler::poll->Got Message: ");
          UART_msg_t_utils::print_msg(msg);
      #endif

      return msg;
     }
    //如果以上分支都不满足（比如 _recv_len = 0），说明既没收到有效字节，也没有缓存的部分数据包，返回初始化的空消息
    return empty_msg;
}

//检查串口是否有可用数据
inline uint8_t UARTHandler::check_for_data()
{
    return MY_SERIAL.available();
}

void UARTHandler::_pack(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data, uint8_t *data_to_pack)
{
    //Pack metadata
    data_to_pack[COMMAND] = msg_id;
    data_to_pack[JOINT_ID] = joint_id;
    
    //Pack payload with platform-specific encoding.
#if UART_PACK_FLOATS
    uint8_t _num_bytes = sizeof(float);
#else
    uint8_t _num_bytes = sizeof(UART_DATA_TYPE);
    uint8_t buf[_num_bytes];
#endif
    for (int i=0; i<len; i++)
    {
        uint8_t _offset = (DATA_START) + _num_bytes*i;
#if UART_PACK_FLOATS
        memcpy((data_to_pack + _offset), (uint8_t*)&data[i], _num_bytes);
#else
        utils::float_to_short_fixed_point_bytes(data[i], buf, FIXED_POINT_FACTOR);
        memcpy((data_to_pack + _offset), buf, _num_bytes);
#endif
    }
}

UART_msg_t UARTHandler::_unpack(uint8_t* data, uint8_t len)
{
    UART_msg_t msg;
    msg.command = data[COMMAND];
    msg.joint_id = data[JOINT_ID];
    float _total_len = len*sizeof(uint8_t);
    float _meta_len = sizeof(msg.command)+sizeof(msg.joint_id);
#if UART_UNPACK_FLOATS
    float _bytes_per = sizeof(float);
    msg.len = (uint8_t)((_total_len - _meta_len) / _bytes_per);
#else
    float _bytes_per = sizeof(UART_DATA_TYPE);
    msg.len = (uint8_t)((_total_len - _meta_len) / _bytes_per);
#endif

    //Fill msg.data, converting the payload to floats as needed.
    for (int i=0; i<msg.len; i++)
    {
        uint8_t _data_offset = (DATA_START) + (i * (uint8_t)_bytes_per);
#if UART_UNPACK_FLOATS
        float tmp = 0;
        memcpy(&tmp, (uint8_t*)data + _data_offset, sizeof(float));
        msg.data[i] = tmp;
#else
        float tmp = 0;
        utils::short_fixed_point_bytes_to_float((uint8_t*)data+_data_offset, &tmp, FIXED_POINT_FACTOR);
        msg.data[i] = tmp;
#endif
    }

    return msg;
}

uint8_t UARTHandler::_get_packed_length(uint8_t msg_id, uint8_t len, uint8_t joint_id, float *data)
{
    uint8_t _val = 0;
#if UART_PACK_FLOATS
    _val += (float)len * sizeof(float);
#else
    //We are converting from float to short int, we must multiply by the size difference
    _val += (float)len * sizeof(UART_DATA_TYPE);
#endif
    _val += sizeof(msg_id);
    _val += sizeof(joint_id); 
    return _val;
}


void UARTHandler::_send_char(uint8_t val)
{
  #if DEBUG_UART_HANDLER
      logger::print("UARTHandler::_send_char->Sending: 0x");
      logger::println(val);
  #endif

  MY_SERIAL.write(val);
}

uint8_t UARTHandler::_recv_char(void)
{  
  uint8_t _data = MY_SERIAL.read();

  #if DEBUG_UART_HANDLER
    logger::print("UARTHandler::_recv_char->Read: "); logger::println(_data);
  #endif

  return _data;
}

/* 发送数据包：从指针 p 指向的起始位置，发送长度为 len 字节的数据   SEND_PACKET: sends a packet of length "len", starting at location "p". */
void UARTHandler::_send_packet(uint8_t* p, uint8_t len)
{
  /*  
  先发送一个 END 字符，清空接收端因线路噪声可能残留的旧数据   
  Send an initial END character to flush out any data that may have accumulated in the receiver due to line noise 
  */
  _send_char(END);

  /*
  遍历数据包中的每个字节，按协议发送对应的字符序列   
  
  For each byte in the packet, send the appropriate character sequence 
  */
  while (len--) {
    switch (*p) {
      /* 
      如果当前字节恰好是 END 字符，则发送两个字符：ESC（0333）+ ESC_END（0334）。
      这样做的目的是告诉接收端："这不是数据包的结束，而是真正的END字符数据"。
  
      If it's the same code as an END character, we send a special two character code so as not to make the receiver think we sent an END 
      */
      case END:
        _send_char(ESC);
        _send_char(ESC_END);
        break;

      /* 
      如果当前字节恰好是 ESC 字符，则发送两个字符：ESC（0333）+ ESC_ESC（0335）。
      这告诉接收端："这不是转义字符，而是真正的ESC字符数据
      
      If it's the same code as an ESC character, we send a special two character code so as not to make the receiver think we sent an ESC 
      */
      case ESC:
        _send_char(ESC);
        _send_char(ESC_ESC);
        break;

      /* 
      非特殊字符正常发送

      Otherwise, we just send the character 
      */
      default:
        //logger::print("UARTHandler::_send_packet->Sending: 0x"); logger::println(*p);
        _send_char(*p);
    }

    p++;
  }

  /* 
  将长度为len的数据发送完毕后, 发送数据包发送完成标志符

  Tell the receiver that we're done sending the packet 
  */
  _send_char(END);
}

// 接收数据包：将数据存入指针 p 指向的缓冲区。
// 若接收字节数超过 len，数据包会被截断。
// 返回实际存入缓冲区的字节数。
/*
返回值：
正数：成功接收到完整数据包，返回实际接收到的字节数
-1：当前未接收新数据但之前有部分数据，结合END标志构成完整消息
0：超时未检测到END标志，已接收数据被缓存至部分数据包
*/

/* RECV_PACKET: receives a packet into the buffer located at "p".
           If more than len bytes are received, the packet will
           be truncated.
           Returns the number of bytes stored in the buffer.
*/
int UARTHandler::_recv_packet(uint8_t *p, uint8_t len)
{
  uint8_t c;          // 存放当前从串口读到的1个字节
  int received = 0;   // 已经接收到的【有效数据字节数】
  int bytes_left;     //函数未使用此变量
  
  _time_left(1);      //执行启动/重置超时计时 
  while (_time_left())//// 在超时前一直循环接收数据
  {
    //检查串口是否有数据可读
    if (!check_for_data()) 
    {
      continue;   // 串口没数据，就一直循环等待
    }

    // 有数据，读出1个字节到c
    c = _recv_char();

    #if DEBUG_UART_HANDLER
        logger::print("UARTHandler::_recv_packet->Got char: ");
        logger::println(c);
    #endif

    //处理所读取的字节数据  Handle bytestuffing if necessary
    switch (c) 
    {

      //// 如果是END字符，表示数据包结束    If it's an END character then we're done with the packet
      case END:
        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::_recv_packet->END CASE");
        #endif

        if (received)
        {
            #if DEBUG_UART_HANDLER
                logger::print("UARTHandler::_recv_packet->Returning: ");
                logger::println(received);
            #endif
            // 1. 已经收到有效数据 → 遇到END = 一整包接收完成
            return received;    // 返回有效数据长度
        }
        //由于UARTHandler类定义为单例模式, 所以其他函数或者代码部分对_partial_packet_len的影响只有在调用_reset_partial_packet() 才会被清零, 否则这个影响会一直存在
        else if (_partial_packet_len)
        {
            #if DEBUG_UART_HANDLER
                logger::print("UARTHandler::_recv_packet->Returning because of _partial_packet: ");
                logger::println(_partial_packet_len);
            #endif
            // 2. 之前缓存了半包数据 → 遇到END = 这包完整了
            return -1;  // 用-1告诉上层：包完整
        }
        else
        {   
           // 3. 开头的同步END（发送方先发的那个）→ 直接忽略
            break;
        }

      /* 
      如果接收到的字符与转义符（ESC）的编码相同，则先等待并读取下一个字符，
      然后根据该字符来确定需要将什么内容存入数据包中 
      */
      case ESC:
        //再读一个字节的数据
        c = _recv_char();

        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::_recv_packet->ESC CASE");
            logger::print("UARTHandler::_recv_packet->ESC Char: ");
            logger::println(c);
        #endif

        /* If "c" is not one of these two, then we have a protocol violation.  The best bet seems to be to leave the byte alone and just stuff it into the packet */
        switch (c) 
        {
          //数据为END字符的情况
          case ESC_END:
              #if DEBUG_UART_HANDLER
                logger::println("UARTHandler::_recv_packet->ESC_END");
              #endif
            c = END;
            break;
          //数据为ESC字符的情况
          case ESC_ESC:
              #if DEBUG_UART_HANDLER
                logger::println("UARTHandler::_recv_packet->ESC_ESC");
              #endif

            c = ESC;
            break;
        }

      default:
        #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::_recv_packet->Default CASE");
        #endif

        if (received < len)
        {
          #if DEBUG_UART_HANDLER
            logger::println("UARTHandler::_recv_packet->Added to buffer");
          #endif

          p[received++] = c;
        }
    }
  }

 //未在超时时间内收到完整消息，将当前数据保存，用于后续拼接重组   There was a timeout before a full message was recieved, save the data to be reconstructed later
  int prior_packet_len = _partial_packet_len;
  _partial_packet_len += received;

  #if DEBUG_UART_HANDLER
      logger::println("UARTHandler::_recv_packet->Timeout!");
      logger::print("UARTHandler::_recv_packet->Saved Bytes: "); 
      logger::print("Prior Packet Length: "); 
      logger::print(prior_packet_len);
      logger::print("\t");
      logger::print("Received: ");
      logger::print(received);
      logger::print("\t");
      logger::print("Partial Packet Length: ");
      logger::println(_partial_packet_len);
  #endif

  // 把本次收到的数据追加到半包缓存区
  for (int i=0; i<(_partial_packet_len); i++)
  {
    _partial_packet[i+prior_packet_len] = p[i];

    #if DEBUG_UART_HANDLER
        logger::print(_partial_packet[i]); logger::println(", ");
    #endif
  }

  #if DEBUG_UART_HANDLER
    logger::println();
  #endif

  return 0;   // 返回0，标记为超时半包
}

/*
这个函数是 UARTHandler 类中的一个私有辅助函数，用于实现超时检测机制
*/
uint8_t UARTHandler::_time_left(uint8_t should_latch)
{
    static float _start_time;
    if (should_latch)
    {
      _start_time = micros();

      #if DEBUG_UART_HANDLER
          logger::print("UARTHandler::_time_left->Latching on ");
          logger::println(_start_time);
      #endif
    }
    float del_t = micros() - _start_time;

    //返回 del_t <= _timeout_us 的结果, 判断是否超时
    return (del_t <= _timeout_us);
}

//清空半缓存包数组
void UARTHandler::_reset_partial_packet()
{
  memset(_partial_packet, 0, _partial_packet_len);
  _partial_packet_len = 0;
}
