

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
#include "SendBulkChar.h"
/**
 * @brief Sends the contents of the txBuffer_bulkStr (the assembled CSV message) 
 * over the primary serial port and clears the buffer afterward.
 * * IMPORTANT: Serial.begin() must be called in setup() before this function runs.
 */
/**
@brief 通过主串口发送 txBuffer_bulkStr 里的内容（已拼接完成的 CSV 消息），
发送完毕后清空该缓冲区。
重要：此函数运行之前，必须先在 setup () 中调用 Serial.begin () 初始化串口。

此函数是外骨骼系统中关键的通信环节，
负责将存储在txBuffer_bulkStr中的控制器参数列表发送给Nano板。
这些参数包含了外骨骼各个关节控制器的配置信息
*/
void send_bulk_char() {
	Serial8.begin(115200);
	pinMode(13,OUTPUT);
	//13号引脚为teensy板载指示灯所在引脚，此处用于指示数据收发状态
	digitalWrite(13,HIGH);
	long initial_time = millis();
	bool NanoReachedOut = false;
	while (millis() - initial_time < 9000)
	{	//  从串口8读取1个字节（Nano发过来的字符）
		if (Serial8.available() > 0) {
			//灯带Nano响应
			digitalWrite(13,LOW);
            char incomingChar = Serial8.read();
			//  判断收到的是不是字符 'R', R是Nano与teensy之间通讯的握手信号	Serial.println(incomingChar);
			if (incomingChar == 'R') {
				//如果收到握手信号, 打印提示：Nano已准备好接收控制器参数
				Serial.print("\nNano confirmed to the Teensy that it’s ready to receive the controller parameter list.");
				while (Serial8.available() > 0) {
					incomingChar = Serial8.read();//clear the buffer, as the Nano might have sent many "R"
				}
				delay(50);	// 小延时，保证通信稳定
				break;
			}
		}
	}
	// delay(5000);
	// digitalWrite(13,LOW);
	// delay(4000);
	// digitalWrite(13,HIGH);
    // C语言字符串的标准约定, 检查缓冲区是否为空	1. Check if the buffer is empty
    if (txBuffer_bulkStr[0] == '\0') {
        Serial.println("Warning: txBuffer_bulkStr is empty. Skipping UART send.");
        return;
    }
    
    // Calculate the exact length of the message.
    size_t message_length = strlen(txBuffer_bulkStr);
    
    // 2. Transmit the entire message in one burst using Serial.write().
    // This is the most efficient method for large C-strings on Arduino.
    Serial8.write(txBuffer_bulkStr, message_length);
	//char myString[] = "f,This is a test char string.\nNew line starts here.,z";
	//Serial8.write(txBuffer_bulkStr, message_length);
	//串口信息发送完毕
    digitalWrite(13,HIGH);
	delay(50);
	// 可选操作：若接收端处理数据的速度较慢，可添加一段短延时。
    // delay(5); 

	//打印已发送内容，并打印提示信息
    Serial.println("\n--- Controller parameter list has been sent. ---");
	Serial.print(txBuffer_bulkStr);

    // 3. 清空缓冲区，为下一次数据拼接做准备。	3. Clear the buffer to prepare for the next assembly.
    // 这一步至关重要，可避免新的、更短的消息中残留旧数据片段。	Crucial to prevent new, shorter messages from containing old data fragments.
    txBuffer_bulkStr[0] = '\0';
	digitalWrite(13,LOW);
	//关闭串口连接
	Serial8.end();
}
#endif