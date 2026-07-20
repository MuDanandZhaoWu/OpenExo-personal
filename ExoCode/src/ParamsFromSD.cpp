#include "ParamsFromSD.h"
#include "Logger.h"
//#define SD_PARAM_DEBUG 1

#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)

    namespace
    {
        bool discard_current_line(File& file)
        {
            while (file.available())
            {
                if (file.read() == '\n')
                {
                    return true;
                }
            }
            return false;
        }

        bool seek_parameter_set(File& file, uint8_t header_size,
                                uint8_t set_num, uint8_t& parameter_count,
                                float* parsed_parameters)
        {
            if (parsed_parameters == nullptr)
            {
                return false;
            }
            for (uint8_t i = 0; i < controller_defs::max_parameters; ++i)
            {
                parsed_parameters[i] = 0.0f;
            }

            if (header_size < 2)
            {
                return false;
            }

            parameter_count = 0;
            const uint16_t lines_to_skip =
                (uint16_t)header_size + (uint16_t)set_num;
            for (uint16_t line = 0; line < lines_to_skip; ++line)
            {
                if (line == 1)
                {
                    const long count = file.parseInt();
                    if (count <= 0 || count > controller_defs::max_parameters)
                    {
                        return false;
                    }
                    parameter_count = (uint8_t)count;
                }
                if (!discard_current_line(file))
                {
                    return false;
                }
            }

            // A newline at the end of the final existing preset is not proof
            // that another preset follows it. Validate the target row before
            // allowing callers to parse values from it.
            if (!file.available())
            {
                return false;
            }
            const unsigned long target_start = file.position();
            String target_line = file.readStringUntil('\n');
            target_line.trim();
            if (target_line.length() == 0)
            {
                return false;
            }

            // Parse every declared field exactly once into a temporary buffer.
            // Callers commit this buffer only after the complete row validates,
            // so malformed/truncated rows cannot partially update live values.
            uint16_t token_start = 0;
            for (uint8_t i = 0; i < parameter_count; ++i)
            {
                const int comma = target_line.indexOf(',', token_start);
                const uint16_t token_end = comma < 0
                    ? (uint16_t)target_line.length()
                    : (uint16_t)comma;
                String token = target_line.substring(token_start, token_end);
                token.trim();
                if (token.length() == 0)
                {
                    return false;
                }
                char* parse_end = nullptr;
                const char* token_text = token.c_str();
                const float value = strtof(token_text, &parse_end);
                if (parse_end == token_text || *parse_end != '\0' ||
                    !isfinite(value))
                {
                    return false;
                }
                parsed_parameters[i] = value;
                if (comma < 0)
                {
                    if (i + 1 < parameter_count)
                    {
                        return false;
                    }
                    break;
                }
                token_start = (uint16_t)comma + 1;
            }

            if (!file.seek(target_start))
            {
                return false;
            }
            return true;
        }

        uint8_t parameter_file_error(uint8_t joint_id)
        {
            return utils::update_bit(
                utils::get_joint_type(joint_id), 1,
                param_error::file_not_found_idx);
        }

        bool parameter_count_is_compatible(JointData* joint_data,
                                           uint8_t controller_id,
                                           uint8_t declared_count)
        {
            if (joint_data == nullptr)
            {
                return false;
            }
            const uint8_t expected_count =
                joint_data->controller.get_parameter_length(controller_id);
            if (declared_count == expected_count && expected_count > 0)
            {
                return true;
            }

            // Preserve the repository's four legacy spline CSV files. They
            // declare 15 fields while the existing spline data model exposes
            // 16; the historical loader likewise populated only the first 15.
            return expected_count == controller_defs::spline::num_parameter &&
                   declared_count == 15;
        }
    }

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
        uint8_t error_type = 0;     // 错误码存储变量   Error message holder

       
        float parsed_parameters[controller_defs::max_parameters] = {0};
        JointData* target_joint = exo_data == nullptr
            ? nullptr
            : exo_data->get_joint_with(joint_id);
        if (target_joint == nullptr ||
            target_joint->controller.get_parameter_length(controller_id) == 0)
        {
            return parameter_file_error(joint_id);
        }

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
                    param_file.setTimeout(10);

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : ");
                        logger::print(filename.c_str());
                        logger::println(" opened");
                    #endif

                    //File 类重载了布尔判断, 所以可以进行文件是否成功打开的判断     Check file exists
                    if (param_file && param_file.available())
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
                            if (!seek_parameter_set(
                                    param_file, header_size, set_num,
                                    param_num_in_file, parsed_parameters) ||
                                !parameter_count_is_compatible(
                                    target_joint, controller_id,
                                    param_num_in_file))
                            {
                                error_type = parameter_file_error(joint_id);
                                param_file.close();
                                return error_type;
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
                                        read_val = parsed_parameters[param_num];
                                        
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
                                        read_val = parsed_parameters[param_num];

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
                    param_file.setTimeout(10);

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : ");
                        logger::print(filename.c_str());
                        logger::println(" opened");
                    #endif

                    //Check file exists
                    if (param_file && param_file.available())
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
                            if (!seek_parameter_set(
                                    param_file, header_size, set_num,
                                    param_num_in_file, parsed_parameters) ||
                                !parameter_count_is_compatible(
                                    target_joint, controller_id,
                                    param_num_in_file))
                            {
                                error_type = parameter_file_error(joint_id);
                                param_file.close();
                                return error_type;
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
                                        read_val = parsed_parameters[param_num];
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
                                        read_val = parsed_parameters[param_num];

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
                    param_file.setTimeout(10);

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : ");
                        logger::print(filename.c_str());
                        logger::println(" opened");
                    #endif

                    //Check file exists
                    if (param_file && param_file.available())
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
                            if (!seek_parameter_set(
                                    param_file, header_size, set_num,
                                    param_num_in_file, parsed_parameters) ||
                                !parameter_count_is_compatible(
                                    target_joint, controller_id,
                                    param_num_in_file))
                            {
                                error_type = parameter_file_error(joint_id);
                                param_file.close();
                                return error_type;
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
                                        read_val = parsed_parameters[param_num];

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
                                        read_val = parsed_parameters[param_num];

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
                    param_file.setTimeout(10);

                    #ifdef SD_PARAM_DEBUG
                        logger::print("set_controller_params : ");
                        logger::print(filename.c_str());
                        logger::println(" opened");
                    #endif

                    //Check file exists
                    if (param_file && param_file.available())
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
                            if (!seek_parameter_set(
                                    param_file, header_size, set_num,
                                    param_num_in_file, parsed_parameters) ||
                                !parameter_count_is_compatible(
                                    target_joint, controller_id,
                                    param_num_in_file))
                            {
                                error_type = parameter_file_error(joint_id);
                                param_file.close();
                                return error_type;
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
                                        read_val = parsed_parameters[param_num];

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
                                        read_val = parsed_parameters[param_num];

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
                    param_file.setTimeout(10);

                    if (param_file && param_file.available())
                    {
                        uint8_t param_num = 0;
                        float read_val = 0;
                        while (param_file.available())
                        {
                            header_size = param_file.parseInt();
                            if (!seek_parameter_set(
                                    param_file, header_size, set_num,
                                    param_num_in_file, parsed_parameters) ||
                                !parameter_count_is_compatible(
                                    target_joint, controller_id,
                                    param_num_in_file))
                            {
                                error_type = parameter_file_error(joint_id);
                                param_file.close();
                                return error_type;
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
                                        read_val = parsed_parameters[param_num];
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
                                        read_val = parsed_parameters[param_num];
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
                    param_file.setTimeout(10);

                    if (param_file && param_file.available())
                    {
                        uint8_t param_num = 0;
                        float read_val = 0;
                        while (param_file.available())
                        {
                            header_size = param_file.parseInt();
                            if (!seek_parameter_set(
                                    param_file, header_size, set_num,
                                    param_num_in_file, parsed_parameters) ||
                                !parameter_count_is_compatible(
                                    target_joint, controller_id,
                                    param_num_in_file))
                            {
                                error_type = parameter_file_error(joint_id);
                                param_file.close();
                                return error_type;
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
                                        read_val = parsed_parameters[param_num];
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
                                        read_val = parsed_parameters[param_num];
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
