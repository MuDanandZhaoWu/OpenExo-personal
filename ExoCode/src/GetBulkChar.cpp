#if defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)
//#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
#include "Arduino.h"
#include <string.h>
#include "GetBulkChar.h"


// --- 共享全局常量（必须与发送端保持一致)       Shared Global Constants (MUST MATCH SENDER) ---
// 定义接收消息缓冲区的最大容量     Define the maximum size for the incoming message buffer.
//const int MAX_MESSAGE_SIZE = 25000; 

// --- 接收端变量       Receiver Variables ---
//char rxBuffer_bulkStr[25000]; // Buffer to store the received data payload
char rxBuffer_bulkStr[MAX_MESSAGE_SIZE]; // 用于存储接收到的数据载荷的缓冲区        Buffer to store the received data payload
int rxIndex = 0;                 // 当前写入 rxBuffer_bulkStr 的索引位置        Current index for writing into rxBuffer_bulkStr
bool messageComplete = false;    // 标记：是否已收到一条完整的消息      Flag indicating a complete message is ready

// State tracking for the serial reception
enum RxState {
    WAITING_FOR_F,      // 等待起始字符 'f'（寻找消息的开头）   Looking for the start character 'f'
    WAITING_FOR_COMMA,  // 已找到 'f'，等待第一个分隔符 ','     Found 'f', now looking for the first delimiter ','
    RECEIVING_DATA      // 正在接收完整的消息帧数据     Receiving the entire message frame
};

//默认初始状态, 寻找其实字符"f"
RxState currentState = WAITING_FOR_F;


// --- 新增函数：阻塞式消息读取器       New Function: Blocking Message Reader ---
/**
 * @brief Synchronously reads the entire message frame ("f,data,z") from UART.
 * This function blocks indefinitely until a full message is received and processed.
 */

/**
 * @brief 从串口(UART)同步读取完整消息帧（格式："f,数据,z"）。
 * 该函数会一直阻塞，直到接收到并处理完一整条完整消息。
 */
 void readSingleMessageBlocking() {
	long initialTime = millis();
    //Serial.println("\n--- Entering Blocking Read Mode ---");
    //Serial.println("System will halt execution until a full frame is received.");
    Serial1.begin(115200);
	//delay(3000);
	
    char txBuffer_NanoReady[2] = "R";
	size_t message_length = strlen(txBuffer_NanoReady);
    // Transmit the entire message in one burst using Serial.write().
    // This is the most efficient method for large C-strings on Arduino.
    // 使用 Serial.write () 一次性批量发送完整消息。
    // 这是 Arduino 中处理长 C 语言字符串最高效的方式。
    while (!Serial1.available()) {  //如果串口空闲, 则执行循环
        //向另一端的设备发送字符"R"，表示本端已准备就绪，请求对方发送数据
		Serial1.write(txBuffer_NanoReady, message_length);
		//Serial.print("\nCharacter R sent.");
        //当串口空闲时, LED发紫光(品红), 表示灯带握手信号
		digitalWrite(LEDR, HIGH);
		digitalWrite(LEDG, LOW);
		digitalWrite(LEDB, HIGH);
		delay(20);
        //超时跳出循环处理
		if (millis() - initialTime > 10000) {
			break;
		}
	}
	
    //退出等待接收数据状态, 此时只亮红灯
	digitalWrite(LEDR, HIGH);
	digitalWrite(LEDG, LOW);
	digitalWrite(LEDB, LOW);
	
    // 循环会一直执行，直到 messageComplete 标志被置为 true 才停止      The loop runs indefinitely until the messageComplete flag is set to true.
    while (!messageComplete) {
        // 只有当串口有数据时才执行循环     Only proceed if data is available in the UART buffer
        while (Serial1.available()) {
			//Serial.print("\nSerial.available() > 0, incomingChar:");
            char incomingChar = Serial1.read();
			//Serial.println(incomingChar);

            // State 1: WAITING_FOR_F
            if (currentState == WAITING_FOR_F) {
                if (incomingChar == 'f') {
                    // 保存字符f并转换currentState到等待WAITING_FOR_COMMA(即',')的状态      Store 'f' and transition
                    if (rxIndex < MAX_MESSAGE_SIZE - 1) {
                        rxBuffer_bulkStr[rxIndex++] = incomingChar;
                        currentState = WAITING_FOR_COMMA;
                    }
					else {
                        // 首个字符即发生缓冲区溢出     Buffer overflow on first character
                        currentState = WAITING_FOR_F;
                        rxIndex = 0;
                    }
                }
            }
            
            // State 2: WAITING_FOR_COMMA
            else if (currentState == WAITING_FOR_COMMA) {
                if (incomingChar == ',') {
                    // Found the first delimiter: Store ',' and begin RECEIVING_DATA
                    if (rxIndex < MAX_MESSAGE_SIZE - 1) {
                        rxBuffer_bulkStr[rxIndex++] = incomingChar;
                        currentState = RECEIVING_DATA;
                        // rxIndex is now 2 (pointing to the start of the payload)
                    }
					else {
                        // Buffer overflow protection: reset and wait again for 'f'
                        //Serial.println("ERROR: Receive buffer overflow during start marker.");
                        currentState = WAITING_FOR_F;
                        rxIndex = 0;
                    }
                }
				else {
                    // If 'f' was followed by something other than ',', discard 'f' and restart
                    currentState = WAITING_FOR_F;
                    rxIndex = 0; // Discard the already stored 'f'
                }
            } 
            
            // State 3: RECEIVING_DATA
            else if (currentState == RECEIVING_DATA) {
                
                // Check for buffer overflow first
                if (rxIndex >= MAX_MESSAGE_SIZE - 1) {
                    //Serial.println("ERROR: Receive buffer overflow.");
                    currentState = WAITING_FOR_F;
                    rxIndex = 0;
                    continue; // Skip processing this character
                }
                
                // --- 1. CHECK FOR END MARKER (",z") ---
                // We check the incoming character against the previous one stored in the buffer.
                if (incomingChar == '?' && rxBuffer_bulkStr[rxIndex - 1] == '?' && rxBuffer_bulkStr[rxIndex - 2] == ',') {
                    
                    // Store the 'z'
                    rxBuffer_bulkStr[rxIndex++] = incomingChar;
                    
                    // Add the null terminator after 'z'
                    rxBuffer_bulkStr[rxIndex] = '\0'; 
                        
                    messageComplete = true; // Exit the blocking while loop
                        
                    // Continue to the processing block below
                }

                // --- 2. STORE DATA ---
                rxBuffer_bulkStr[rxIndex] = incomingChar;
                rxIndex++;
            }
			// 3. Immediately exit the inner 'while' loop if the message is complete
            if (messageComplete) {
                break;
            }
        }
		if (millis() - initialTime > 8000) {
			break;
		}
        // Optional: Introduce a small delay if no data is available to prevent watchdog timer resets on some boards.
        // delay(1); 
    } // End of while (!messageComplete)
	
	Serial1.end();
    // Reset the state machine to be ready for the *next* time this function is called (if ever)
    // Note: Since this is in setup(), we typically won't run again, but it's clean practice.
    // messageComplete = false; // We don't reset this, as it's the loop exit condition.
    currentState = WAITING_FOR_F;
    rxIndex = 0;
	
	digitalWrite(LEDR, HIGH);
	digitalWrite(LEDG, HIGH);
	digitalWrite(LEDB, HIGH);
}

#endif