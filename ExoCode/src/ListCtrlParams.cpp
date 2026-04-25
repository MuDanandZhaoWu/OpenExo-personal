

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
#include "ListCtrlParams.h"

char txBuffer_bulkStr[MAX_MESSAGE_SIZE];

/*
0. 函数核心为读取控制器参数、组织为数组

1. 尝试初始化SD卡连接(SD.begin())

2. 初始化计数器和索引变量，用于跟踪处理进度
   设置失败计数器 failed2open 来统计无法打开的文件数量

3. 使用循环遍历12个关节（从左踝关节到右臂2）
   根据配置信息判断哪些关节被使用以及它们的默认控制器设置
   对每个有效的关节和控制器组合，从SD卡读取相应的CSV配置文件

4. 从CSV文件中读取第五行数据（通常是参数设置行）
   解析关节名称、控制器名称等信息
   将解析后的参数按规范存储到二维数组 stringArray 中

5. 调用 create_csv_message() 将所有参数整理成字符串消息格式
*/
void ctrl_param_array_gen(uint8_t* config_to_send) {
	//SD.begin(SD_SELECT) 初始化SD卡模块与微控制器之间的通信接口
	if (!SD.begin(SD_SELECT)) {
			while (1)	//这里的while似乎缺少了大括号(这样只控制只控制紧接着while(1)的下一句)，导致初始化连接SD卡失败后无法如预期进入打印错误信息的循环
			
			if (Serial)
			{
				// logger::print("SD.begin() failed");
				// logger::print("\n");
				Serial.println("SD.begin() failed");
			}
	}
	
	//输出最大消息大小常量的值到串口监视器
	// Serial.print("\nconst size_t MAX_MESSAGE_SIZE = ");
	// Serial.print(MAX_MESSAGE_SIZE);
	
	uint8_t csvCount;	//存储当前处理的关节类型的控制器数量
	uint8_t row_idx = 0;	//存储当前处理的CSV文件的行索引
	failed2open = 0;	//用于统计未能成功打开的CSV配置文件数量
	
	//遍历所有可能的关节类型(12种)，并根据配置信息确定哪些关节需要处理，然后为这些关节读取所有相关的控制器参数文件
	for (int i_joint = 1; i_joint < 13; i_joint++) {
		switch (i_joint)
		{
		case 1://left ankle
		//先判断踝关节是否被禁用, 再判断外骨骼配置是否支持左踝关节(可能支持双踝关节或者只支持做踝关节)
		if ((config_to_send[config_defs::exo_ankle_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]))))
		{	//确定控制器数量
			csvCount = (uint8_t)config_defs::ankle_controllers::Count;
		}
		else {
			continue;	//若不满足条件,跳过此关节,检查下一个
		}
			break;
		case 2://right ankle
		if ((config_to_send[config_defs::exo_ankle_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::ankle_controllers::Count;
		}
		else {
			continue;
		}
			break;	
		case 3://left hip
		if ((config_to_send[config_defs::exo_hip_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::hip_controllers::Count;
		}
		else {
			continue;
		}
			break;
		case 4://right hip
		if ((config_to_send[config_defs::exo_hip_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::hip_controllers::Count;
		}
		else {
			continue;
		}
			break;
		case 5://left knee
		if ((config_to_send[config_defs::exo_knee_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::knee_controllers::Count;
		}
		else {
			continue;
		}
			break;
		case 6://right knee
		if ((config_to_send[config_defs::exo_knee_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::knee_controllers::Count;
		}
		else {
			continue;
		}
			break;
		case 7://left elbow
		if ((config_to_send[config_defs::exo_elbow_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::elbow_controllers::Count;
		}
		else {
			continue;
		}
			break;
		case 8://right elbow
		if ((config_to_send[config_defs::exo_elbow_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::elbow_controllers::Count;
		}
		else {
			continue;
		}
			break;
		case 9://left arm 1
		if ((config_to_send[config_defs::exo_arm_1_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::arm_1_controllers::Count;
		}
		else {
			continue;
		}
			break;
		case 10://right arm 1
		if ((config_to_send[config_defs::exo_arm_1_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::arm_1_controllers::Count;
		}
		else {
			continue;
		}
			break;
		case 11://left arm 2
		if ((config_to_send[config_defs::exo_arm_2_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::left == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::arm_2_controllers::Count;
		}
		else {
			continue;
		}
			break;
		case 12://right arm 2
		if ((config_to_send[config_defs::exo_arm_2_default_controller_idx] > 1) && ((((uint8_t)config_defs::exo_side::bilateral == config_to_send[config_defs::exo_side_idx])) || (((uint8_t)config_defs::exo_side::right == config_to_send[config_defs::exo_side_idx]))))
		{
			csvCount = (uint8_t)config_defs::arm_2_controllers::Count;
		}
		else {
			continue;
		}
			break;
		}
		
	
		//Configure
		//Serial.print("\n\n\n\nTotal number of controllers: ");
		//Serial.print(csvCount);
		
		
		int start_ctrl = 2; // Skip disabled controller for all joints.	跳过跳过ID=1的 disabled 控制器
		for (int i_ctrl = start_ctrl; i_ctrl < csvCount; i_ctrl++) {
			bool csvExists;		//检查是否存在对应的CSV参数文件
			std::string filename;	//存储控制器参数文件的路径名
			char joint_id_string;	//用于存储关节ID字符串
			switch (i_joint)
			{
			case 1://left ankle
				//joint_id_string_l = (uint8_t)config_defs::joint_id::left_ankle;
				joint_id_val = (uint8_t)config_defs::joint_id::left_ankle;	//为每个关节设置唯一的数值ID
				strncpy(jointName, "Ankle(L)", 10); 
				jointName[9] = '\0';
				// 检查当前控制器是否有对应的参数文件
				// 使用 count() 方法检查是否存在指定控制器ID的CSV文件映射, count()函数是检查map()容器中对应映射的key值是否存在 (这个key值是定义在parseIni.h文件枚举中的控制器ID)
				// 如果存在，则获取文件路径
				csvExists = controller_parameter_filenames::ankle.count(i_ctrl);
				if (csvExists) {
					//将对应控制器所在的文件路径存储在filename变量中
					filename = controller_parameter_filenames::ankle[i_ctrl];
				}
				break;
			case 2://right ankle
				//joint_id_string_l = (uint8_t)config_defs::joint_id::left_ankle;
				joint_id_val = (uint8_t)config_defs::joint_id::right_ankle;
				strncpy(jointName, "Ankle(R)", 10); 
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::ankle.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::ankle[i_ctrl];
				}
				break;
			case 3://left hip
				//joint_id_string_l = (uint8_t)config_defs::joint_id::left_hip;
				joint_id_val = (uint8_t)config_defs::joint_id::left_hip;
				strncpy(jointName, "Hip(L)", 10); 
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::hip.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::hip[i_ctrl];
				}
				break;
			case 4://right hip
				//joint_id_string_l = (uint8_t)config_defs::joint_id::left_hip;
				joint_id_val = (uint8_t)config_defs::joint_id::right_hip;
				strncpy(jointName, "Hip(R)", 10); 
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::hip.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::hip[i_ctrl];
				}
				break;
			case 5://left knee
				//joint_id_string_l = (uint8_t)config_defs::joint_id::left_knee;
				joint_id_val = (uint8_t)config_defs::joint_id::left_knee;
				strncpy(jointName, "Knee(L)", 10); 
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::knee.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::knee[i_ctrl];
				}
				break;
			case 6://right knee
				//joint_id_string_l = (uint8_t)config_defs::joint_id::left_knee;
				joint_id_val = (uint8_t)config_defs::joint_id::right_knee;
				strncpy(jointName, "Knee(R)", 10); 
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::knee.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::knee[i_ctrl];
				}
				break;
			case 7://left elbow
				//joint_id_string_l = (uint8_t)config_defs::joint_id::left_elbow;
				joint_id_val = (uint8_t)config_defs::joint_id::left_elbow;
				strncpy(jointName, "Elbow(L)", 10); 
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::elbow.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::elbow[i_ctrl];
				}
				break;
			case 8://right elbow
				//joint_id_string_l = (uint8_t)config_defs::joint_id::left_elbow;
				joint_id_val = (uint8_t)config_defs::joint_id::right_elbow;
				strncpy(jointName, "Elbow(R)", 10); 
				// Ensure the array is null-terminated at the end of its allocated space
				// to prevent printing garbage if the source string was 10 chars long.
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::elbow.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::elbow[i_ctrl];
				}
				break;
			case 9://left arm 1
				joint_id_val = (uint8_t)config_defs::joint_id::left_arm_1;
				strncpy(jointName, "Arm1(L)", 10);
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::arm_1.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::arm_1[i_ctrl];
				}
				break;
			case 10://right arm 1
				joint_id_val = (uint8_t)config_defs::joint_id::right_arm_1;
				strncpy(jointName, "Arm1(R)", 10);
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::arm_1.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::arm_1[i_ctrl];
				}
				break;
			case 11://left arm 2
				joint_id_val = (uint8_t)config_defs::joint_id::left_arm_2;
				strncpy(jointName, "Arm2(L)", 10);
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::arm_2.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::arm_2[i_ctrl];
				}
				break;
			case 12://right arm 2
				joint_id_val = (uint8_t)config_defs::joint_id::right_arm_2;
				strncpy(jointName, "Arm2(R)", 10);
				jointName[9] = '\0';
				csvExists = controller_parameter_filenames::arm_2.count(i_ctrl);
				if (csvExists) {
					filename = controller_parameter_filenames::arm_2[i_ctrl];
				}
				break;
			} 
			
			if (csvExists) { // condition is true if count is 1
				//Serial.print("\n\nController ");
				//Serial.print((int)i_ctrl);
				//Serial.println(" has a csv.");

				const char* filename_char = filename.c_str();
				
				// 调用函数读取并解析第五行数据 Call the function to read and parse the fifth row	
				int columnsRead = readAndParseFifthRow(filename_char, stringArray, MAX_COLUMNS, MAX_STRING_LENGTH, row_idx, i_ctrl);
				

				// Print the results
				if (columnsRead > 0) {
					//Serial.println("\nFifth Row Data Saved:");
					/* for (int i = 0; i < columnsRead; i++) {
					  Serial.print("\nParam ");
					  Serial.print(i + 1);
					  Serial.print(": ");
					  Serial.print(stringArray[row_idx][i]);
					} */
					row_idx++;
				}
				else {
					//Serial.println("\nFailed to read or parse the fifth row.");
					failed2open++;
				}
			}
		}
	}
	
	create_csv_message();
	// The message is now ready for transmission/printing
	/* Serial.println("--- Prepared CSV Message ---");
	Serial.println(txBuffer_bulkStr);
	
	Serial.print("\nNominal total number of csv: ");
	Serial.print((uint8_t)config_defs::ankle_controllers::Count + (uint8_t)config_defs::hip_controllers::Count + (uint8_t)config_defs::knee_controllers::Count + (uint8_t)config_defs::elbow_controllers::Count);
	Serial.print("\nNumber of row in stringArray: ");
	Serial.print(row_idx+1);
	Serial.print("\nNumber of failed 2 open csv: ");
	Serial.print(failed2open); */
}

// --- Function to Read and Parse ---
/*这个函数主要用于OpenExo外骨骼控制系统中，
从SD卡上的CSV配置文件中读取第五行的控制器参数。
系统需要动态加载不同关节（髋、膝、踝、肘等）和不同控制器类型的参数，
并将这些参数按照特定格式组织，以便后续传输到其他设备或模块进行处理。

根据项目规范，这些参数用于初始化不同的控制器算法，
如零扭矩模式、PID控制、特定的步态辅助算法等，
使系统能够根据不同任务和用户需求动态切换控制策略
*/
 int readAndParseFifthRow(
    const char* filename_char, 
    char arr[][MAX_COLUMNS][MAX_STRING_LENGTH], 
    int maxCols, 
    int maxLen, 
    uint8_t row_idx,
	int i_ctrl) 
{
    //用于存储解析出的参数组件的缓冲区 Buffers to hold extracted components
    char joint_name[MAX_NAME_LENGTH];
    char controller_name[MAX_NAME_LENGTH];

    // 尝试从文件路径中解析出关节名称和控制器名称	Attempt to extract Joint and Controller names from the path
    bool extraction_success = retrieveJointAndController(
        filename_char, 
        joint_name, 
        controller_name
    );
    
    // 如果从文件路径中提取关节和控制器名称的操作失败，则使用默认值		Use default values if extraction fails
    //uint8_t joint_id_val = 255; // Default UNKNOWN ID, 废弃代码, 以值255表示未知ID, 保留是保留注释是为了便于后续维护者理解原始设计意图
    const char* controller_str = "UNKNOWN_CTRL";

    if (extraction_success) {
        //joint_id_val = get_joint_id_from_name(joint_name);
        controller_str = controller_name;
    }
    
    // 检查数组是否有足够的空间用于插入前缀列(当前前缀列定义为四列)		Check if there is enough space in the array for the prefix
    if (maxCols < PREFIX_COLS) {
        Serial.println("\nERROR: MAX_COLUMNS too small for prefix insertion.");
        return 0;
    }

	//调试打印：告知正在打开的具体CSV文件
    //Serial.print("\nOpening csv: ");
    //Serial.print(filename_char);
    
    //使用SD库打开文件，如果打开失败则返回0 	 Using SD.h types (File)
    File dataFile = SD.open(filename_char);
    
    if (!dataFile) {
        Serial.print("\nError opening ");
        Serial.println(filename_char);
        // Assuming 'failed2open' is a global variable
        // failed2open++;
        return 0; // Return 0 columns read	表示本次 CSV 文件解析读取到的有效列数为 0
    }

    // 1. Find the Fifth Line
    int rowCount = 0;	//行计数器
    String targetLine = "";		//存储CSV文件目标行(此处即第五行)的完整内容
    
    // Read byte by byte until the end of the file or the fifth row is found
	//dataFile.available()为SD 库的核心文件读取判断函数, 返回值为true时表示文件指针后还有未读取的字节，false则表示读到文件末尾
    while (dataFile.available()) {
        char c = dataFile.read();	//dataFile.read()是SD 库的逐字节读取函数，每次调用从文件中读取一个字符，并将文件指针后移一位
        
		/*
		只有当行计数器rowCount=4时（即程序识别到当前正在读取第五行），
		才会将读取到的字符c拼接到targetLine；

		前四行的字符会被读取，但不会做任何存储处理，
		仅用于更新行计数器，保证只保留第五行的有效数据，节省内存。
		*/
        if (rowCount == 4) { // Row 5 is index 4 (0-based)
            targetLine += c;
        }
        
		// 遇到换行符，代表当前行读取完成，行计数器+1
        if (c == '\n') {
            rowCount++;
            if (rowCount > 4) {
                break; // 第五行读取完成后，立即跳出循环，停止读取文件	Stop after reading the entire fifth row
            }
        }
    }

    dataFile.close(); // Always close the file!

    if (rowCount < 4) {
        return 0;
    }
    
    // 2. Parse the Line (Tokenize the String) - STARTING AT INDEX PREFIX_COLS (3)
    int colIndex = PREFIX_COLS; // Start parsing data into index 3
    int charIndex = 0;
    int dataColsRead = 0; // Tracks columns successfully parsed from the file

    for (int i = 0; i < targetLine.length(); i++) {
        char c = targetLine.charAt(i);

        if (c == ',') {
            // End of a field: terminate the current string and move to the next column
            arr[row_idx][colIndex][charIndex] = '\0'; // Null-terminate the string
            
            dataColsRead++;
            colIndex++;
            charIndex = 0; // Reset character index for the next column
            
            // Safety check: stop if max columns reached (accounting for the 3 prefixes)
            if (colIndex >= maxCols) break;

        }
        else if (c != '\r' && c != '\n') { 
            // Append character to the current string
            if (charIndex < maxLen - 1) { 
                arr[row_idx][colIndex][charIndex] = c;
                charIndex++;
            }
        }
    }
    
    // 3. Handle the Last Parsed Column
    if (colIndex < maxCols && charIndex > 0) {
        arr[row_idx][colIndex][charIndex] = '\0';
        dataColsRead++;
        colIndex++;
    } 
    // Ensure the column immediately after the last written data is also null-terminated (cleared)
    else if (colIndex < maxCols) {
        arr[row_idx][colIndex][0] = '\0';
    }


    // --- 4. INSERT PREFIX COLUMNS (Columns 0, 1, and 2) ---
	
	// New Column 0: Insert char Joint name string
	strncpy(arr[row_idx][0], jointName, maxLen - 1);
	arr[row_idx][0][maxLen - 1] = '\0';
	
    // Column 0: Insert uint8_t Joint ID value
    // Use snprintf to convert the uint8_t (%u) into a string
    snprintf(arr[row_idx][1], maxLen, "%u", joint_id_val);

    // Column 1: Insert Controller Name string
    strncpy(arr[row_idx][2], controller_str, maxLen - 1);
    arr[row_idx][2][maxLen - 1] = '\0';

    // Column 2: Insert the full File Name
    //strncpy(arr[row_idx][2], filename_char, maxLen - 1);
	snprintf(arr[row_idx][3], maxLen, "%u", i_ctrl);
    //arr[row_idx][3][maxLen - 1] = '\0';


    // The total number of columns written is the prefix columns plus the data columns read
    int totalColsWritten = PREFIX_COLS + dataColsRead;
    
    //Serial.print("\nNumber of columns read: ");
    //Serial.print(dataColsRead);
    //Serial.print(" (Total stored: ");
    //Serial.print(totalColsWritten);
    //Serial.print(")");

    return totalColsWritten; 
}

/*
 函数的作用是创建一个CSV格式的消息，
 将存储在二维数组 stringArray 中的参数数据组装成一个大的字符串，
 并将其存储到全局缓冲区 txBuffer_bulkStr 中
 */
void create_csv_message() {
    // 1. 初始化缓冲区用于批量数据发送的字符串缓冲区	Initialize the buffer
    txBuffer_bulkStr[0] = '\0'; // Start with an empty string
	//strcat为字符串拼接函数, 添加消息前缀 "f,"，这在项目规范中被定义为消息的起始标志
	strcat(txBuffer_bulkStr, "f,");
	
    // 2. 遍历所有已存储的行（参数快照）	Iterate through all stored rows (snapshots)
    for (int i = 0; i < MAX_SNAPSHOTS; i++) {
        
        // Safety Check: Stop if the row is empty (based on our placeholder logic)
        if (stringArray[i][0][0] == '\0') {
            break; 
        }

        // 3. Iterate through all columns in the current row
        for (int j = 0; j < MAX_COLUMNS; j++) {
            
			if (stringArray[i][j][0] == '\0') {
				break; 
			}
			
            // a. 将数组单元格中的字符串拼接到发送缓冲区	Append the string from the cell
            //注意：此操作依赖于stringArray[i][j]以空字符终止 	Note: This relies on stringArray[i][j] being null-terminated
            strcat(txBuffer_bulkStr, stringArray[i][j]);

            // b. 如果不是最后一列，并且下一列不为空，则添加逗号分隔符到发送缓冲区	Append the comma delimiter, except after the last column
            if (j < MAX_COLUMNS - 1) {
				if (stringArray[i][j+1][0] != '\0') {
					strcat(txBuffer_bulkStr, ",");
				}
            }
        }
        
        // 4. 添加行结束符	Append the End-of-Line symbol
        // // 通常使用'\n'作为换行符；若需兼容Windows系统/蓝牙低功耗（BLE），可改用"\r\n"	Using '\n' (newline) is common; use "\r\n" for Windows/BLE compatibility if needed.
        strcat(txBuffer_bulkStr, "\n"); 
    }
	strcat(txBuffer_bulkStr, ",?");		//最后添加结束标志 ",?"，这在项目规范中被定义为消息的结束符
}

/**
 * @brief Extracts the "Joint" and "Controller" names from a path string 
 * formatted as "\Joint\Controller.csv". (Logic based on extract_components.cpp)
 */
/*
一个字符串解析函数，
用于从形如 "/Joint/Controller.csv" 的路径字符串中
提取关节（Joint）和控制器（Controller）名称

输入: 一个包含文件路径的字符串 (filename_char)
输出: 两个字符串缓冲区 (joint_out,controller_out)，用于存储解析出的关节名称和控制器名称
*/
bool retrieveJointAndController(const char* filename_char, char* joint_out, char* controller_out) {
	//首先检查所有输入参数是否有效，若有任一参数为空指针则直接返回 false
	if (!filename_char || !joint_out || !controller_out) return false;

    // 1. PREPARE START POINTER
    const char* start_ptr = filename_char;
	//跳过路径开头的斜杠字符 '/'，使指针指向关节名称的起始位置。
    if (*start_ptr == '/') {
        start_ptr++; 
    }
    
    // 2. FIND JOINT END ('/')
	/*
	查找第一个 '/' 字符，它标志着关节名称的结束和控制器名称的开始。
	如果找不到该字符，则返回 false
	*/
    const char* joint_end_ptr = strchr(start_ptr, '/');
    if (!joint_end_ptr) return false;

    // 3. EXTRACT JOINT NAME
    size_t joint_len = joint_end_ptr - start_ptr;
	// 计算安全拷贝长度：实际长度未超缓冲区则取实际值，超则取最大长度 - 1（预留 \0 终止符）
    size_t copy_len = (joint_len < MAX_NAME_LENGTH) ? joint_len : MAX_NAME_LENGTH - 1;
    strncpy(joint_out, start_ptr, copy_len);
    joint_out[copy_len] = '\0'; 

    // 4. FIND CONTROLLER END ('.csv')
    const char* controller_start_ptr = joint_end_ptr + 1;
    const char* controller_end_ptr = strstr(controller_start_ptr, ".csv");
	//如果未找到.csv则返回false
    if (!controller_end_ptr) return false;
    
    // 5. EXTRACT CONTROLLER NAME
    size_t controller_len = controller_end_ptr - controller_start_ptr;
    copy_len = (controller_len < MAX_NAME_LENGTH) ? controller_len : MAX_NAME_LENGTH - 1;
    strncpy(controller_out, controller_start_ptr, copy_len);
    controller_out[copy_len] = '\0'; 

    return true;
}

#endif
