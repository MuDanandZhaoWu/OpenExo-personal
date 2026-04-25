#include "ParamsFromSD.h"
#include "Logger.h"
//#define SD_PARAM_DEBUG 1

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

    void print_param_error_message(uint8_t error_type)
    {
        //logger::print(utils::get_is_left(error_type)? "Left " : "Right ");
        switch (error_type & ((uint8_t)config_defs::joint_id::hip | (uint8_t)config_defs::joint_id::knee | (uint8_t)config_defs::joint_id::ankle | (uint8_t)config_defs::joint_id::elbow | (uint8_t)config_defs::joint_id::arm_1 | (uint8_t)config_defs::joint_id::arm_2))
        {
            case (uint8_t)config_defs::joint_id::hip:
                //logger::print("Hip ");    
                break;
            case (uint8_t)config_defs::joint_id::knee:
                //logger::print("Knee ");
                break;
            case (uint8_t)config_defs::joint_id::ankle:
                //logger::print("Ankle ");
                break;
            case (uint8_t)config_defs::joint_id::elbow:
                //logger::print("Elbow ");
                break;
            case (uint8_t)config_defs::joint_id::arm_1:
                //logger::print("Arm 1 ");
                break;
            case (uint8_t)config_defs::joint_id::arm_2:
                //logger::print("Arm 2 ");
                break;
        }
        //检查错误类型中的特定位，以确定发生了哪种类型的错误
        if (utils::get_bit(error_type, param_error::SD_not_found_idx))
        {
            //logger::print("SD Not Found, ");
        }            

        //这里似乎是一个Bug, get_bit的输入应该是 file_not_found_idx，否则永远不会判断 “文件找不到”
        if (utils::get_bit(error_type, param_error::SD_not_found_idx))
        {
            //logger::print("File Not Found, ");
        } 
        //logger::println("File Not Found, ");
    }
    
    /**
     * @brief 从SD卡上的参数文件中加载控制器参数，并将其设置到ExoData结构体中
     * 
     * 此函数根据关节ID和控制器ID从SD卡中读取指定参数集，并更新相应的控制器参数。
     * 支持多种关节类型：髋关节、膝关节、踝关节、肘关节、臂1和臂2。
     * 
     * @param joint_id 关节ID，用于确定要配置哪个关节
     * @param controller_id 控制器ID，用于确定要使用的参数文件
     * @param set_num 要读取的参数集编号
     * @param exo_data 指向ExoData结构体的指针，用于存储外骨骼数据
     * @return uint8_t 错误码，如果成功则为0，否则为相应的错误类型
     */
    uint8_t set_controller_params(uint8_t joint_id, uint8_t controller_id, uint8_t set_num, ExoData* exo_data)
    {   
        //SD 类继承自 Stream 类，后者提供了许多我们会用到的实用方法     SD inherits from stream which has a lot more useful methods that we will use.
        File param_file;            // 定义SD卡文件操作对象，用于打开/读取/关闭参数文件
        std::string filename;       // 存储参数文件的文件名（字符串格式）
        uint8_t header_size;        // 参数前需要跳过的表头行数     Number of lines to skip before the parameters
        uint8_t param_num_in_file;  // 文件中待读取的参数数量   Number of parameters to pull in
        uint8_t line_to_read;       // 读取参数的目标行     Line to read the parameters from
        uint8_t error_type = 0;     // 错误码存储变量   Error message holder

       
        switch(utils::get_joint_type(joint_id))
        {
            case (uint8_t)config_defs::joint_id::hip:
            {
                #ifdef SD_PARAM_DEBUG
                    logger::println("\n\nset_controller_params : Hip");
                #endif

                //初始化SPI总线, 准备与SD卡通讯(项目SD卡使用SPI协议进行通信)     Connect to SD card
                SPI.begin();

                #ifdef SD_PARAM_DEBUG
                    logger::println("set_controller_params : SPI Begin");
                #endif

                //检查SD卡是否初始化成功, 若不成功, 返回对应的错误类型
                if (!SD.begin(SD_SELECT))
                {
                    error_type = utils::update_bit((uint8_t)config_defs::joint_id::hip, 1, param_error::SD_not_found_idx);
                    
                    #ifdef SD_PARAM_DEBUG
                        logger::println("set_controller_params : SD Not Found");
                    #endif
                    
                    return error_type;
                }
                else 
                {
                    //如果SD卡初始化成功, 读取对应关节的对应控制器文件名称      Get filename
                    filename = controller_parameter_filenames::hip[controller_id];

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : filename = ");
                        logger::println(filename.c_str());
                    #endif

                    /*
                    将filename转换为C语言风格, 以「只读模式(FILE_READ模式)」打开 SD 卡上指定名字的文件，
                    并把打开后的文件对象赋值给 param_file(SD.open(...) 返回的就是一个 File 对象)。
                    
                    后续就可以用 param_file 读取文件里的控制器参数Open File
                    */
                    param_file = SD.open(filename.c_str(), FILE_READ);

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : ");
                        logger::print(filename.c_str());
                        logger::println(" opened");
                    #endif

                    //File 类重载了布尔判断, 所以可以进行文件是否成功打开的判断     Check file exists
                    if (param_file)
                    {   
                        while(param_file.available())
                        {
                            //First value should be header size
                            header_size = param_file.parseInt();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : header size ");
                                logger::println(header_size);
                            #endif

                            //Skip to the line we need
                            line_to_read = header_size + set_num;
                            for (int line_being_read = 0; line_being_read < line_to_read; line_being_read++)
                            { 
                                //First value in second line should be parameter number
                                if (line_being_read == 1)
                                {
                                    param_num_in_file = param_file.parseInt();

                                    #ifdef SD_PARAM_DEBUG
                                        logger::print("set_controller_params : Number of parameters in file = ");
                                        logger::println(param_num_in_file);
                                    #endif    
                                }

                                //Keep going through the file till the next new line. This is so it will restart if timeout happens.          
                                while(!param_file.findUntil('\n','\n'))
                                {
                                    ;
                                }

                                #ifdef SD_PARAM_DEBUG
                                    logger::print("set_controller_params : read line ");
                                    logger::println(line_being_read);
                                #endif
                            }
                            
                            //Store the line start value so we can go back here
                            unsigned long line_start = param_file.position();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : parameter set start ");
                                logger::println(line_start);
                            #endif
                            
                            //Find the end of the line
                            param_file.readStringUntil('\n');
                            unsigned long line_end = param_file.position();
                            
                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : parameter set end ");
                                logger::println(line_end);
                            #endif
                            
                            //Reset to the start of the line
                            param_file.seek(line_start);

                            #ifdef SD_PARAM_DEBUG
                                logger::println("set_controller_params : reset to line start");
                            #endif

                            //Set the parameters.
                            uint8_t param_num = 0;
                            float read_val = 0;
                            if(utils::get_is_left(joint_id))
                            {
                                #ifdef SD_PARAM_DEBUG
                                    logger::println("set_controller_params : is Left ");
                                #endif
                                
                                //Read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                        
                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("+Value in file :\t");
                                            logger::println(read_val);
                                        #endif  
                                    }
                                    else
                                    {
                                        read_val = 0;

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("-File Line Ended :\t");
                                            logger::println(read_val);
                                        #endif
                                    }
                                    
                                    exo_data->left_side.hip.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            else
                            {
                                #ifdef SD_PARAM_DEBUG
                                    logger::println("set_controller_params : is Right ");
                                #endif
                                
                                //Read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("+Value in file :\t");
                                            logger::println(read_val);
                                        #endif 
                                    }
                                    else
                                    {
                                        read_val = 0;

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("-File Line Ended :\t");
                                            logger::println(read_val);
                                        #endif
                                    }
                                    
                                    exo_data->right_side.hip.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            //We don't need to read the rest of the file
                            break;
                        }
                        
                    }
                    else
                    { 
                        error_type = utils::update_bit((uint8_t)config_defs::joint_id::hip, 1, param_error::file_not_found_idx);

                        #ifdef SD_PARAM_DEBUG
                            logger::println("set_controller_params : File not found");
                        #endif
                    }
                    param_file.close();

                    #ifdef SD_PARAM_DEBUG
                        logger::println("set_controller_params : File Closed");
                    #endif
                }
                break;
            }
            case (uint8_t)config_defs::joint_id::knee:
            {
                #ifdef SD_PARAM_DEBUG
                    logger::println("\n\nset_controller_params : Knee");
                #endif

                //Connect to SD card
                SPI.begin();

                #ifdef SD_PARAM_DEBUG
                    logger::println("set_controller_params : SPI Begin");
                #endif

                if (!SD.begin(SD_SELECT))
                {
                    error_type = utils::update_bit((uint8_t)config_defs::joint_id::knee, 1, param_error::SD_not_found_idx);

                    #ifdef SD_PARAM_DEBUG
                        logger::println("set_controller_params : SD Not Found");
                    #endif

                    return error_type;
                }
                else 
                {
                    //Get filename
                    filename = controller_parameter_filenames::knee[controller_id];

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : filename = ");
                        logger::println(filename.c_str());
                    #endif

                    //Open File
                    param_file = SD.open(filename.c_str(), FILE_READ);

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : ");
                        logger::print(filename.c_str());
                        logger::println(" opened");
                    #endif

                    //Check file exists
                    if (param_file)
                    {   
                        while(param_file.available())
                        {
                            //First value should be header size
                            header_size = param_file.parseInt();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : header size ");
                                logger::println(header_size);
                            #endif

                            //Skip to the line we need
                            line_to_read = header_size + set_num;
                            for (int line_being_read = 0; line_being_read < line_to_read; line_being_read++)
                            { 
                                //First value in second line should be parameter number
                                if (line_being_read == 1)
                                {
                                    param_num_in_file = param_file.parseInt();

                                    #ifdef SD_PARAM_DEBUG
                                        logger::print("set_controller_params : Number of parameters in file = ");
                                        logger::println(param_num_in_file);
                                    #endif    
                                } 

                                //Keep going through the file till the next new line.  This is so it will restart if timeout happens.          
                                while(!param_file.findUntil('\n','\n'))
                                {
                                    ;
                                }

                                #ifdef SD_PARAM_DEBUG
                                    logger::print("set_controller_params : read line ");
                                    logger::println(line_being_read);
                                #endif
                            }
                            
                            //Store the line start value so we can go back here
                            unsigned long line_start = param_file.position();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : parameter set start ");
                                logger::println(line_start);
                            #endif
                            
                            //Find the end of the line
                            param_file.readStringUntil('\n');
                            unsigned long line_end = param_file.position();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : parameter set end ");
                                logger::println(line_end);
                            #endif
                            
                            //Reset to the start of the line
                            param_file.seek(line_start);

                            #ifdef SD_PARAM_DEBUG
                                logger::println("set_controller_params : reset to line start");
                            #endif

                            //Set the parameters.
                            uint8_t param_num = 0;
                            float read_val = 0;
                            if(utils::get_is_left(joint_id))
                            {
                                #ifdef SD_PARAM_DEBUG
                                    logger::println("set_controller_params : is Left ");
                                #endif
                                
                                //Read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("+Value in file :\t");
                                            logger::println(read_val);
                                        #endif   
                                    }
                                    else
                                    {
                                        read_val = 0;

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("-File Line Ended :\t");
                                            logger::println(read_val);
                                        #endif
                                    }
                                    
                                    exo_data->left_side.knee.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            else
                            {
                                #ifdef SD_PARAM_DEBUG
                                    logger::println("set_controller_params : is Right ");
                                #endif
                                
                                //Read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("+Value in file :\t");
                                            logger::println(read_val);
                                        #endif
                                        
                                    }
                                    else
                                    {
                                        read_val = 0;

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("-File Line Ended :\t");
                                            logger::println(read_val);
                                        #endif
                                    }
                                    
                                    exo_data->right_side.knee.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            //We don't need to read the rest of the file
                            break;
                        }
                        
                    }
                    else
                    { 
                        error_type = utils::update_bit((uint8_t)config_defs::joint_id::knee, 1, param_error::file_not_found_idx);

                        #ifdef SD_PARAM_DEBUG
                            logger::println("set_controller_params : File not found");
                        #endif
                    }
                    param_file.close();

                    #ifdef SD_PARAM_DEBUG
                        logger::println("set_controller_params : File Closed");
                    #endif
                }
                break;
            }
            case (uint8_t)config_defs::joint_id::ankle:
            {
                #ifdef SD_PARAM_DEBUG
                    logger::println("\n\nset_controller_params : Ankle");
                #endif

                //Connect to SD card
                SPI.begin();

                #ifdef SD_PARAM_DEBUG
                    logger::println("set_controller_params : SPI Begin");
                #endif

                if (!SD.begin(SD_SELECT))
                {
                    error_type = utils::update_bit((uint8_t)config_defs::joint_id::ankle, 1, param_error::SD_not_found_idx);

                    #ifdef SD_PARAM_DEBUG
                        logger::println("set_controller_params : SD Not Found");
                    #endif

                    return error_type;
                }
                else 
                {
                    //Get filename
                    filename = controller_parameter_filenames::ankle[controller_id];

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : filename = ");
                        logger::println(filename.c_str());
                    #endif

                    //Open File
                    param_file = SD.open(filename.c_str(), FILE_READ);

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : ");
                        logger::print(filename.c_str());
                        logger::println(" opened");
                    #endif

                    //Check file exists
                    if (param_file)
                    {   
                
                        //Set the parameters.
                        uint8_t param_num = 0;
                        float read_val = 0;
                        while(param_file.available())
                        {
                            //First value should be header size
                            header_size = param_file.parseInt();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : header size ");
                                logger::println(header_size);
                            #endif

                            //Skip to the line we need
                            line_to_read = header_size + set_num;
                            for (int line_being_read = 0; line_being_read < line_to_read; line_being_read++)
                            { 
                                //First value in second line should be parameter number
                                if (line_being_read == 1)
                                {
                                    param_num_in_file = param_file.parseInt();

                                    #ifdef SD_PARAM_DEBUG
                                        logger::print("set_controller_params : Number of parameters in file = ");
                                        logger::println(param_num_in_file);
                                    #endif    
                                } 

                                //Keep going through the file till the next new line.  This is so it will restart if timeout happens.          
                                while(!param_file.findUntil('\n','\n'))
                                {
                                    ;
                                }

                                #ifdef SD_PARAM_DEBUG
                                    logger::print("set_controller_params : read line ");
                                    logger::println(line_being_read);
                                #endif
                            }
                            
                            //Store the line start value so we can go back here
                            unsigned long line_start = param_file.position();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : parameter set start ");
                                logger::println(line_start);
                            #endif
                            
                            //Find the end of the line
                            param_file.readStringUntil('\n');
                            unsigned long line_end = param_file.position();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : parameter set end ");
                                logger::println(line_end);
                            #endif
                            
                            //Reset to the start of the line
                            param_file.seek(line_start);

                            #ifdef SD_PARAM_DEBUG
                                logger::println("set_controller_params : reset to line start");
                            #endif
                            
                            if(utils::get_is_left(joint_id))
                            {
                                #ifdef SD_PARAM_DEBUG
                                    logger::println("set_controller_params : is Left ");
                                #endif
                                
                                //Read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("+Value in file :\t");
                                            logger::println(read_val);
                                        #endif
                                        
                                    }
                                    else
                                    {
                                        read_val = 0;

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("-File Line Ended :\t");
                                            logger::println(read_val);
                                        #endif
                                    }
                                    
                                    exo_data->left_side.ankle.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            else
                            {
                                #ifdef SD_PARAM_DEBUG
                                    logger::println("set_controller_params : is Right ");
                                #endif
                                
                                //Read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("+Value in file :\t");
                                            logger::println(read_val);
                                        #endif
                                        
                                    }
                                    else
                                    {
                                        read_val = 0;

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("-File Line Ended :\t");
                                            logger::println(read_val);
                                        #endif
                                    }
                                    
                                    exo_data->right_side.ankle.controller.parameters[param_num] = read_val;
                                    
                                    param_num++;
                                }
                            }
                            //We don't need to read the rest of the file
                            break;
                        }
                        
                    }
                    else
                    { 
                        error_type = utils::update_bit((uint8_t)config_defs::joint_id::ankle, 1, param_error::file_not_found_idx);

                        #ifdef SD_PARAM_DEBUG
                            logger::println("set_controller_params : File not found");
                        #endif
                    }
                    param_file.close();

                    #ifdef SD_PARAM_DEBUG
                        logger::println("set_controller_params : File Closed");
                    #endif
                }
                break;
            }
            case (uint8_t)config_defs::joint_id::elbow:
            {
                #ifdef SD_PARAM_DEBUG
                    logger::println("\n\nset_controller_params : Elbow");
                #endif

                //Connect to SD card
                SPI.begin();

                #ifdef SD_PARAM_DEBUG
                    logger::println("set_controller_params : SPI Begin");
                #endif

                if (!SD.begin(SD_SELECT))
                {
                    error_type = utils::update_bit((uint8_t)config_defs::joint_id::elbow, 1, param_error::SD_not_found_idx);

                    #ifdef SD_PARAM_DEBUG
                        logger::println("set_controller_params : SD Not Found");
                    #endif

                    return error_type;
                }
                else
                {
                    //Get filename
                    filename = controller_parameter_filenames::elbow[controller_id];

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : filename = ");
                        logger::println(filename.c_str());
                    #endif

                    //Open File
                    param_file = SD.open(filename.c_str(), FILE_READ);

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : ");
                        logger::print(filename.c_str());
                        logger::println(" opened");
                    #endif

                    //Check file exists
                    if (param_file)
                    {

                        //Set the parameters.
                        uint8_t param_num = 0;
                        float read_val = 0;
                        while (param_file.available())
                        {
                            //First value should be header size
                            header_size = param_file.parseInt();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : header size ");
                                logger::println(header_size);
                            #endif

                            //Skip to the line we need
                            line_to_read = header_size + set_num;
                            for (int line_being_read = 0; line_being_read < line_to_read; line_being_read++)
                            {
                                //First value in second line should be parameter number
                                if (line_being_read == 1)
                                {
                                    param_num_in_file = param_file.parseInt();

                                    #ifdef SD_PARAM_DEBUG
                                        logger::print("set_controller_params : Number of parameters in file = ");
                                        logger::println(param_num_in_file);
                                    #endif    
                                }

                                //Keep going through the file till the next new line.  This is so it will restart if timeout happens.          
                                while (!param_file.findUntil('\n', '\n'))
                                {
                                    ;
                                }

                                #ifdef SD_PARAM_DEBUG
                                    logger::print("set_controller_params : read line ");
                                    logger::println(line_being_read);
                                #endif
                            }

                            //Store the line start value so we can go back here
                            unsigned long line_start = param_file.position();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : parameter set start ");
                                logger::println(line_start);
                            #endif

                            //Find the end of the line
                            param_file.readStringUntil('\n');
                            unsigned long line_end = param_file.position();

                            #ifdef SD_PARAM_DEBUG
                                logger::print("set_controller_params : parameter set end ");
                                logger::println(line_end);
                            #endif

                            //Reset to the start of the line
                            param_file.seek(line_start);

                            #ifdef SD_PARAM_DEBUG
                                logger::println("set_controller_params : reset to line start");
                            #endif

                            if (utils::get_is_left(joint_id))
                            {
                                #ifdef SD_PARAM_DEBUG
                                    logger::println("set_controller_params : is Left ");
                               #endif

                                //Read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("+Value in file :\t");
                                            logger::println(read_val);
                                        #endif

                                    }
                                    else
                                    {
                                        read_val = 0;

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("-File Line Ended :\t");
                                            logger::println(read_val);
                                        #endif
                                    }

                                    exo_data->left_side.elbow.controller.parameters[param_num] = read_val;

                                    param_num++;
                                }
                            }
                            else
                            {
                                #ifdef SD_PARAM_DEBUG
                                    logger::println("set_controller_params : is Right ");
                                #endif

                                //Read till the end of the line or all the parameters are full
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("+Value in file :\t");
                                            logger::println(read_val);
                                        #endif
                                    }
                                    else
                                    {
                                        read_val = 0;

                                        #ifdef SD_PARAM_DEBUG
                                            logger::print("-File Line Ended :\t");
                                            logger::println(read_val);
                                        #endif
                                    }

                                    exo_data->right_side.elbow.controller.parameters[param_num] = read_val;

                                    param_num++;
                                }
                            }
                            //We don't need to read the rest of the file
                            break;
                        }

                    }
                    else
                    {
                        error_type = utils::update_bit((uint8_t)config_defs::joint_id::elbow, 1, param_error::file_not_found_idx);

                        #ifdef SD_PARAM_DEBUG
                            logger::println("set_controller_params : File not found");
                        #endif
                    }
                    param_file.close();

                    #ifdef SD_PARAM_DEBUG
                        logger::println("set_controller_params : File Closed");
                    #endif
                }
                break;
            }
            case (uint8_t)config_defs::joint_id::arm_1:
            {
                SPI.begin();

                if (!SD.begin(SD_SELECT))
                {
                    error_type = utils::update_bit((uint8_t)config_defs::joint_id::arm_1, 1, param_error::SD_not_found_idx);
                    return error_type;
                }
                else
                {
                    filename = controller_parameter_filenames::arm_1[controller_id];
                    param_file = SD.open(filename.c_str(), FILE_READ);

                    if (param_file)
                    {
                        uint8_t param_num = 0;
                        float read_val = 0;
                        while (param_file.available())
                        {
                            header_size = param_file.parseInt();
                            line_to_read = header_size + set_num;
                            for (int line_being_read = 0; line_being_read < line_to_read; line_being_read++)
                            {
                                if (line_being_read == 1)
                                {
                                    param_num_in_file = param_file.parseInt();
                                }
                                while (!param_file.findUntil('\n', '\n'))
                                {
                                    ;
                                }
                            }

                            unsigned long line_start = param_file.position();
                            param_file.readStringUntil('\n');
                            unsigned long line_end = param_file.position();
                            param_file.seek(line_start);
                            (void)line_end;

                            if (utils::get_is_left(joint_id))
                            {
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                    }
                                    else
                                    {
                                        read_val = 0;
                                    }

                                    exo_data->left_side.arm_1.controller.parameters[param_num] = read_val;
                                    param_num++;
                                }
                            }
                            else
                            {
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                    }
                                    else
                                    {
                                        read_val = 0;
                                    }

                                    exo_data->right_side.arm_1.controller.parameters[param_num] = read_val;
                                    param_num++;
                                }
                            }
                            break;
                        }
                    }
                    else
                    {
                        error_type = utils::update_bit((uint8_t)config_defs::joint_id::arm_1, 1, param_error::file_not_found_idx);
                    }
                    param_file.close();
                }
                break;
            }
            case (uint8_t)config_defs::joint_id::arm_2:
            {
                SPI.begin();

                if (!SD.begin(SD_SELECT))
                {
                    error_type = utils::update_bit((uint8_t)config_defs::joint_id::arm_2, 1, param_error::SD_not_found_idx);
                    return error_type;
                }
                else
                {
                    filename = controller_parameter_filenames::arm_2[controller_id];
                    param_file = SD.open(filename.c_str(), FILE_READ);

                    if (param_file)
                    {
                        uint8_t param_num = 0;
                        float read_val = 0;
                        while (param_file.available())
                        {
                            header_size = param_file.parseInt();
                            line_to_read = header_size + set_num;
                            for (int line_being_read = 0; line_being_read < line_to_read; line_being_read++)
                            {
                                if (line_being_read == 1)
                                {
                                    param_num_in_file = param_file.parseInt();
                                }
                                while (!param_file.findUntil('\n', '\n'))
                                {
                                    ;
                                }
                            }

                            unsigned long line_start = param_file.position();
                            param_file.readStringUntil('\n');
                            unsigned long line_end = param_file.position();
                            param_file.seek(line_start);
                            (void)line_end;

                            if (utils::get_is_left(joint_id))
                            {
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                    }
                                    else
                                    {
                                        read_val = 0;
                                    }

                                    exo_data->left_side.arm_2.controller.parameters[param_num] = read_val;
                                    param_num++;
                                }
                            }
                            else
                            {
                                while (param_num < controller_defs::max_parameters)
                                {
                                    if (param_num_in_file > param_num)
                                    {
                                        read_val = param_file.parseFloat();
                                    }
                                    else
                                    {
                                        read_val = 0;
                                    }

                                    exo_data->right_side.arm_2.controller.parameters[param_num] = read_val;
                                    param_num++;
                                }
                            }
                            break;
                        }
                    }
                    else
                    {
                        error_type = utils::update_bit((uint8_t)config_defs::joint_id::arm_2, 1, param_error::file_not_found_idx);
                    }
                    param_file.close();
                }
                break;
            }
            
        }

        #ifdef SD_PARAM_DEBUG
            logger::println("set_controller_params : Never Entered Switch case");
        #endif

        return error_type;
    }

#endif
