#ifndef ListCtrlParams_h
#define ListCtrlParams_h



#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
#include "ExoData.h"
#include "ParseIni.h"
#include "Utilities.h"
#include "ParamsFromSD.h"
#include "PlottingTitles.h"

#include <SPI.h>
#include <SD.h>
#include <map>
#include <string>

#ifndef SD_SELECT
    #define SD_SELECT BUILTIN_SDCARD
#endif

	// // 定义数组的大小以及每个字符串的最大长度
	const int MAX_COLUMNS = 30;		//定义 CSV 表格（或参数列表）中最大列数为 30。
	const int MAX_STRING_LENGTH = 10;	//定义 CSV 单元格中单个字符串的最大长度为 10 个字符
	//MAX_SNAPSHOTS 定义关节控制器参数的最大快照数量（可理解为 CSV 表格的最大行数），数值是所有关节控制器总数的 2 倍
	const int MAX_SNAPSHOTS = 2 * ((uint8_t)config_defs::ankle_controllers::Count + (uint8_t)config_defs::hip_controllers::Count + (uint8_t)config_defs::knee_controllers::Count + (uint8_t)config_defs::elbow_controllers::Count + (uint8_t)config_defs::arm_1_controllers::Count + (uint8_t)config_defs::arm_2_controllers::Count);
	
	//该代码用于计算存储 CSV 格式数据的传输缓冲区的最大字节数，确保缓冲区能够容纳所有关节控制器参数的完整消息（含分隔符、换行符、结束符），避免内存溢出或数据截断
	// 计算传输缓冲区的最大尺寸：
	// (每个单元格的最大字符数 + 1个逗号分隔符) * 最大列数 + 
	// (+ 1个换行符) * 最大快照数 + 
	// (+ 1个字节用于最终的空终止符)
	const size_t MAX_MESSAGE_SIZE = 
		(MAX_STRING_LENGTH + 1) * MAX_COLUMNS * MAX_SNAPSHOTS + MAX_SNAPSHOTS + 1;
	

	
	
	
	

	// Array to hold the strings from the fifth row

void ctrl_param_array_gen(uint8_t* config_to_send);			//生成控制器参数数组
int readAndParseFifthRow(const char* filename_char, char arr[][MAX_COLUMNS][MAX_STRING_LENGTH], int maxCols, int maxLen, uint8_t row_idx, int i_ctrl);
void create_csv_message();
bool retrieveJointAndController(const char* filename_char, char* joint_out, char* controller_out);

// Define txBuffer_bulkStr here too, as it's also global data		这里也定义 txBuffer_bulkStr，因为它也是全局数据
    // The 1D buffer that will hold the final, flattened CSV string		存储最终扁平化CSV字符串的一维缓冲区(将二维数组转化为一维数组后存储)
	extern char txBuffer_bulkStr[MAX_MESSAGE_SIZE];

namespace { // Use an anonymous namespace for file-local scope (Best Practice)
    static char stringArray[MAX_SNAPSHOTS][MAX_COLUMNS][MAX_STRING_LENGTH]; 
    
	uint8_t failed2open;
	// Define the number of prefix columns to insert
	const int PREFIX_COLS = 4;
	const size_t MAX_NAME_LENGTH = 64;
	uint8_t joint_id_val;
	char jointName[10];
}


#endif
#endif
