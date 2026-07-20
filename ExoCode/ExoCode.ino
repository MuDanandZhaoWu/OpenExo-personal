   /*
   Code used to run the exo from the teensy.  This communicates with the nano over UART.
   
   P. Stegall Jan 2022
*/  

//Teensy Operation
#if defined(ARDUINO_TEENSY36) | defined(ARDUINO_TEENSY41)

//UNCOMMENT TO UTILIZE
//#define INCLUDE_FLEXCAN_DEBUG   //Flag to print CAN debugging messages for the motors(用于打印电机CAN调试消息的标志)
//#define MAKE_PLOTS              //Flag to serial plot(用于串行绘图的标志)
//#define MAIN_DEBUG              //Flag to print Arduino debugging statements(用于打印Arduino调试语句的标志)
//#define HEADLESS                //Flag to be used when there is no app access(启用HEADLESS模式可能意味着设备将按照SD卡上的配置文件独立运行，而不需要与PC端的图形应用程序进行实时通信)

//Standard Libraries
#include <stdint.h>
#include <IntervalTimer.h>

//Common Libraries
#include "src/Board.h"
#include "src/ExoData.h"
#include "src/Exo.h"
#include "src/Utilities.h"
#include "src/StatusDefs.h"
#include "src/Config.h"

//Specific Libraries
#include "src/ParseIni.h"
#include "src/ParamsFromSD.h"
#include "src/ListCtrlParams.h"
#include "src/SendBulkChar.h"
#include "src/PlottingTitles.h"

//Board to board coms
#include "src/UARTHandler.h"
#include "src/uart_commands.h"
#include "src/UART_msg_t.h"

//Logging
#include "src/Logger.h"
#include "src/PiLogger.h"

//Array used to store config information(用于存储配置信息的数组, number_of_keys在ParseIni.h文件中定义为71)
namespace config_info
{
    uint8_t (config_to_send)[ini_config::number_of_keys];
}

void setup()
{
    analogReadResolution(12);       //将analogRead()的返回值范围从0-1023变为0-4095
    
    Serial.begin(115200);
    //延迟以等待串口稳定
    //delay(500);

    #ifdef SIMPLE_DEBUG
        Serial.print("\nIn SIMPLE_DEBUG mode, debugging statements are printed.");
        Serial.print("\nProgrammed PCB version: ");
        Serial.print(BOARD_VERSION);
        Serial.print("\nFor a list of UART message index, check UART_command_names in uart_commands.h");
        Serial.print("\nFor a list of the controller and joint ids, check the controller enum classes in Parseini.h (Lines 127-185)");
    #endif

    //从SD卡获取配置信息（调用ParseIni中的函数）
    ini_parser(config_info::config_to_send);              
    
	//Debugging ListCtrlParams
	long initialTime = millis();

    //读取控制器参数、组织为数组
	ctrl_param_array_gen(config_info::config_to_send);  //传入参数为config_info命名空间中的config_to_send数组
    //获取上位机的标题
	create_plotting_titles(config_info::config_to_send);
    //异步串口通信, 将txBuffer_bulkStr缓冲区中的内容发送给Nano
	send_bulk_char();
    /*
    测量启动时间：精确地测量从程序启动到完成
    关键初始化步骤（如读取配置文件、生成参数数组等）所花费的时间。

    调试与优化：帮助开发者了解系统启动的效率，
    识别可能存在的瓶颈，并进行相应的优化。

    日志记录：提供一个清晰的日志条目，便于后续分析和故障排查。
    */
	long time_spent = millis() - initialTime;
	Serial.print("\nTeensy Boot time added: ");
	Serial.print(time_spent);
	
    //  打印配置信息，用于确认配置已正确读取（正常情况下配置值不应为 0）。   Print to confirm config came through correctly (Should not contain zeros).
    #if defined(MAIN_DEBUG) || defined(SIMPLE_DEBUG)
        for (int i = 0; i < ini_config::number_of_keys; i++)
        {
          logger::print("[" + String(i) + "] : " + String((int)config_info::config_to_send[i]) + "\n");
        }
        logger::print("\n");
    #endif
    
    // 绘图时使用的信号名称标签   Labels for the signals if plotting.
    #ifdef MAKE_PLOTS
          logger::print("Left_hip_trq_cmd, ");
          logger::print("Left_hip_current, ");
          logger::print("Right_hip_trq_cmd, ");     //左髋关节扭矩指令
          logger::print("Right_hip_current, ");
          logger::print("Left_ankle_trq_cmd, ");
          logger::print("Left_ankle_current, ");
          logger::print("Right_ankle_trq_cmd, ");
          logger::print("Right_ankle_current, ");
          logger::print("Left_ankle_torque_measure, ");
          logger::print("\n");
      #endif
}

void loop()
{
    static bool first_run = true;
    
    //创建数据对象，入口函数唯一的ExoData实例，保证系统共用同一套数据 Create the data object
    //static关键字保证exo_data在这个程序的声明周期内只创建一次，避免重复初始化和数据冲突
    static ExoData exo_data(config_info::config_to_send);     

    //Print to make sure object was created
    #ifdef MAIN_DEBUG
        if (first_run)
        {
            logger::print("Superloop :: exo_data created"); 
        }
    #endif

    //从这里开始, exo_data实例被一路传递到motor一层, 每一层级的数据继承自它的上一层 Create the exo object
    static Exo exo(&exo_data);                                

    //Print to make sure object was created
    #ifdef MAIN_DEBUG
        if (first_run)
        {
            logger::print("Superloop :: exo created");
        }
    #endif

    /*
    创建 UART 通信处理器的单例实例      Creates instance of UART Handler
        
    static
    表示这个变量是静态的：
    整个程序运行期间只创建一次
    不会随着 loop() 循环反复新建，避免重复初始化串口。
    */
    static UARTHandler* uart_handler = UARTHandler::get_instance();
    
    
    if (first_run)
    {   
        /*
        设置 first_run = false 的原因是为了确保某些初始化代码只在程序
        循环第一次运行时执行一次，而不是每次循环都执行

        这种模式被称为"首次运行模式"(First Run Pattern)
        */
        first_run = false;

        //Waits for the message telling it to get the config information 
        UART_command_utils::wait_for_get_config(uart_handler, &exo_data, UART_times::CONFIG_TIMEOUT);

        //Print detailing which joint and side is used
        /*
        程序会持续监听通信链路（如 UART/CAN 总线），
        直到收到「要求获取配置信息」的指令，再执行后续的配置读取 / 回传操作
        */
        #ifdef MAIN_DEBUG
            logger::print("Superloop :: Start First Run Conditional\n");
            logger::print("Superloop :: exo_data.left_side.hip.is_used = ");
            logger::print(exo_data.left_side.hip.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.right_side.hip.is_used = ");
            logger::print(exo_data.right_side.hip.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.left_side.knee.is_used = ");
            logger::print(exo_data.left_side.knee.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.right_side.knee.is_used = ");
            logger::print(exo_data.right_side.knee.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.left_side.ankle.is_used = ");
            logger::print(exo_data.left_side.ankle.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.right_side.ankle.is_used = ");
            logger::print(exo_data.right_side.ankle.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.left_side.elbow.is_used = ");
            logger::print(exo_data.left_side.elbow.is_used);
            logger::print("\n");
            logger::print("Superloop :: exo_data.right_side.elbow.is_used = ");
            logger::print(exo_data.right_side.elbow.is_used);
            logger::print("\n");
            logger::print("\n");
        #endif
        
        //只调用与已启用的电机相关的函数        Only call functions related to used motors
        //代码的主要目的就是在首次运行（first run）时将所有已启用的关节电机设置为零扭矩（zero_torque）控制器，以实现安全启动
        if (exo_data.left_side.hip.is_used)
        {
            //  开启电机    Turn motor on
            exo_data.left_side.hip.motor.is_on = true;
            
            //确保电机控制增益设为 0，避免电机出现异常、抖动或失控      Make sure motor gains are set to 0 so there is no funny business
            exo_data.left_side.hip.motor.kp = 0;
            exo_data.left_side.hip.motor.kd = 0;

            //Handles desired operations if in headless mode
            // 若处于无头模式，则处理预设的目标操作
            #ifdef HEADLESS

                //Set the controller parameters to their default
                // 读取SD卡中的配置信息, 将控制器参数恢复为默认值
                set_controller_params((uint8_t) exo_data.left_side.hip.id, config_info::config_to_send[config_defs::exo_hip_default_controller_idx], 0, &exo_data); //This function is found in ParamsFromSD
                
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Left Hip Parameters Set");
                #endif
                
                // 等待校准完成后再设置实际控制器
                //Waits until calibration is done to set actual controller
                //exo_data.left_side.hip.controller.controller中, 前一个controller是ControllerData类的一个对象, 后一个ControllerData是ControllerData类中的成员变量
                exo_data.left_side.hip.controller.controller = (uint8_t)config_defs::hip_controllers::zero_torque;  //  以零扭矩模式启动    Start in zero torque
                exo.left_side.get_hip().set_controller(exo_data.left_side.hip.controller.controller);                    //  随后设置为目标控制器, 代码将zero torque控制器应用在左髋关节的电机上    Then sets to desired controller                  
                
            #endif
        }
        
        if (exo_data.right_side.hip.is_used)
        {
            //Turn motor on 
            exo_data.right_side.hip.motor.is_on = true;

            //Make sure motor gains are set to 0 so there is no funny business
            exo_data.right_side.hip.motor.kp = 0;
            exo_data.right_side.hip.motor.kd = 0;

            //Handles desired operations if in headless mode
            #ifdef HEADLESS

                //Set the controller parameters to thier default
                set_controller_params((uint8_t) exo_data.right_side.hip.id, config_info::config_to_send[config_defs::exo_hip_default_controller_idx], 0, &exo_data);
                
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Right Hip Parameters Set");
                #endif
                
                //Waits until calibration is done to set actual controller
                exo_data.right_side.hip.controller.controller = (uint8_t)config_defs::hip_controllers::zero_torque;   //Start in zero torque
                exo.right_side.get_hip().set_controller(exo_data.right_side.hip.controller.controller);                    //Then sets to desired controller
            
            #endif
        }

        if (exo_data.left_side.knee.is_used)
        {
            //Turn motor on
            exo_data.left_side.knee.motor.is_on = true;
            
            //Make sure motor gains are set to 0 so there is no funny business
            exo_data.left_side.knee.motor.kp = 0;
            exo_data.left_side.knee.motor.kd = 0;

            //Handles desired operations if in headless mode
            #ifdef HEADLESS

                //Set the controller parameters to thier default
                set_controller_params((uint8_t) exo_data.left_side.knee.id, config_info::config_to_send[config_defs::exo_knee_default_controller_idx], 0, &exo_data); //This function is found in ParamsFromSD
                
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Left Knee Parameters Set");
                #endif
                
                //Waits until calibration is done to set actual controller
                exo_data.left_side.knee.controller.controller = (uint8_t)config_defs::knee_controllers::zero_torque; //Start in zero torque
                exo.left_side.get_knee().set_controller(exo_data.left_side.knee.controller.controller);                    //Then sets to desired controller                  
                
            #endif
        }
        
        if (exo_data.right_side.knee.is_used)
        {
            //Turn motor on 
            exo_data.right_side.knee.motor.is_on = true;

            //Make sure motor gains are set to 0 so there is no funny business
            exo_data.right_side.knee.motor.kp = 0;
            exo_data.right_side.knee.motor.kd = 0;

            //Handles desired operations if in headless mode
            #ifdef HEADLESS

                //Set the controller parameters to thier default
                set_controller_params((uint8_t) exo_data.right_side.knee.id, config_info::config_to_send[config_defs::exo_knee_default_controller_idx], 0, &exo_data);
                
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Right Knee Parameters Set");
                #endif
                
                //Waits until calibration is done to set actual controller
                exo_data.right_side.knee.controller.controller = (uint8_t)config_defs::knee_controllers::zero_torque;   //Start in zero torque
                exo.right_side.get_knee().set_controller(exo_data.right_side.knee.controller.controller);                    //Then sets to desired controller
            
            #endif
        }

        if (exo_data.left_side.ankle.is_used)
        {
            #ifdef MAIN_DEBUG
              logger::print("Superloop :: Left Ankle Used");
            #endif

            //Turn motor on
            exo_data.left_side.ankle.motor.is_on = true;

            //Make sure motor gains are set to 0 so there is no funny business
            exo_data.left_side.ankle.motor.kp = 0;
            exo_data.left_side.ankle.motor.kd = 0;

            //Handles desired operations if in headless mode
            #ifdef HEADLESS

                //Set the controller parameters to thier default
                set_controller_params((uint8_t) exo_data.left_side.ankle.id, config_info::config_to_send[config_defs::exo_ankle_default_controller_idx], 0, &exo_data);
                
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Left Ankle Parameters Set");
                #endif
                
                //Waits until calibration is done to set actual controller
                exo_data.left_side.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::zero_torque;   //Start in zero torque
                exo.left_side.get_ankle().set_controller(exo_data.left_side.ankle.controller.controller);                      //Then sets to desired controller
                
            #endif
        }
        
        if (exo_data.right_side.ankle.is_used)
        {
            //Turn motor on
            exo_data.right_side.ankle.motor.is_on = true;

            //Make sure motor gains are set to 0 so there is no funny business
            exo_data.right_side.ankle.motor.kp = 0;
            exo_data.right_side.ankle.motor.kd = 0;

            //Handles desired operations if in headless mode
            #ifdef HEADLESS

                //Set the controller parameters to thier default
                set_controller_params((uint8_t) exo_data.right_side.ankle.id, config_info::config_to_send[config_defs::exo_ankle_default_controller_idx], 0, &exo_data);
                
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Right Ankle Parameters Set");
                #endif
                
                //Waits until calibration is done to set actual controller
                exo_data.right_side.ankle.controller.controller = (uint8_t)config_defs::ankle_controllers::zero_torque;   //Start in zero torque
                exo.right_side.get_ankle().set_controller(exo_data.right_side.ankle.controller.controller);                    //Then sets to desired controller
                
            #endif
        }

        if (exo_data.left_side.elbow.is_used)
        {
            #ifdef MAIN_DEBUG
              logger::print("Superloop :: Left Elbow Used");
            #endif

            //Turn motor on
            exo_data.left_side.elbow.motor.is_on = true;

            //Make sure motor gains are set to 0 so there is no funny business
            exo_data.left_side.elbow.motor.kp = 0;
            exo_data.left_side.elbow.motor.kd = 0;

            //Handles desired operations if in headless mode
            #ifdef HEADLESS

                //Set the controller parameters to thier default
                set_controller_params((uint8_t) exo_data.left_side.elbow.id, config_info::config_to_send[config_defs::exo_elbow_default_controller_idx], 0, &exo_data);
                
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Left Elbow Parameters Set");
                #endif
                
                //Waits until calibration is done to set actual controller
                exo_data.left_side.elbow.controller.controller = (uint8_t)config_defs::elbow_controllers::zero_torque;    //Start in zero torque
                exo.left_side.get_elbow().set_controller(exo_data.left_side.elbow.controller.controller);                      //Then sets to desired controller
                
            #endif
        }
        
        if (exo_data.right_side.elbow.is_used)
        {
            //Turn motor on
            exo_data.right_side.elbow.motor.is_on = true;

            //Make sure motor gains are set to 0 so there is no funny business
            exo_data.right_side.elbow.motor.kp = 0;
            exo_data.right_side.elbow.motor.kd = 0;

            //Handles desired operations if in headless mode
            #ifdef HEADLESS

                //Set the controller parameters to thier default
                set_controller_params((uint8_t) exo_data.right_side.elbow.id, config_info::config_to_send[config_defs::exo_elbow_default_controller_idx], 0, &exo_data);
                
                #ifdef MAIN_DEBUG
                  logger::print("Superloop :: Right Elbow Parameters Set");
                #endif
                
                //Waits until calibration is done to set actual controller
                exo_data.right_side.elbow.controller.controller = (uint8_t)config_defs::elbow_controllers::zero_torque;   //Start in zero torque
                exo.right_side.get_elbow().set_controller(exo_data.right_side.elbow.controller.controller);                    //Then sets to desired controller
                
            #endif
        }
        
        // Give the motors time to wake up
        // 给电机留出启动稳定的时间
        #ifdef MAIN_DEBUG
          logger::print("Superloop :: Motor Charging Delay - Please be patient");
        #endif 

        // Set the status to Motor Startup
        // 将状态设置为电机启动中
        exo_data.set_status(status_defs::messages::motor_start_up); 

        // 定义与电机启动延时相关的参数 Define the Parameters involved with motor startup delay
        unsigned int motor_start_delay_ms = 10;                     //Delay duration, previously set to 60000, if you are having issues with startup try using this time instead 
        unsigned int motor_start_time = millis();                   
        unsigned int dot_print_ms = 1000;                           
        unsigned int last_dot_time = millis();                     

        //Loop that gives motors time to wake up
        while (millis() - motor_start_time < motor_start_delay_ms)
        {
            // 更新 LED 状态，用于提示当前正处于延时阶段    Updates LED status to let you know it is in its delay
            exo.status_led.update(exo_data.get_status());

            #ifdef MAIN_DEBUG
              if(millis() - last_dot_time > dot_print_ms)
              {
                last_dot_time = millis();
                logger::print(".");
              } 
            #endif
        }
        
        #ifdef MAIN_DEBUG
          logger::println();  //Just gives some spacing to Serial Monitor while de-bugging
        #endif

        // 若无法通过上位机应用配置系统，则在此处进行配置   Configure the system if you can't set it with the app
        #ifdef HEADLESS
            bool enable_overide = true;
            
            //校准传感器并启用所使用的电机 Calibrates torque sensor and enables motor for each used joint
            if(exo_data.left_side.hip.is_used)
            { 
              exo_data.left_side.hip.calibrate_torque_sensor = true;
              exo_data.left_side.hip.motor.enabled = true;
            }
           
            if(exo_data.right_side.hip.is_used)
            {
              exo_data.right_side.hip.calibrate_torque_sensor = true;
              exo_data.right_side.hip.motor.enabled = true; 
            }

            if(exo_data.left_side.knee.is_used)
            { 
              exo_data.left_side.knee.calibrate_torque_sensor = true;
              exo_data.left_side.knee.motor.enabled = true;
            }
           
            if(exo_data.right_side.knee.is_used)
            {
              exo_data.right_side.knee.calibrate_torque_sensor = true;
              exo_data.right_side.knee.motor.enabled = true; 
            }
            
            if(exo_data.left_side.ankle.is_used)
            {
                exo_data.left_side.ankle.calibrate_torque_sensor = true; 
                exo_data.left_side.ankle.motor.enabled = true;
            }
           
            if(exo_data.right_side.ankle.is_used)
            {
                exo_data.right_side.ankle.calibrate_torque_sensor = true;  
                exo_data.right_side.ankle.motor.enabled = true;
            }

            if(exo_data.left_side.elbow.is_used)
            {
                exo_data.left_side.elbow.calibrate_torque_sensor = true; 
                exo_data.left_side.elbow.motor.enabled = true;
            }
           
            if(exo_data.right_side.elbow.is_used)
            {
                exo_data.right_side.elbow.calibrate_torque_sensor = true;  
                exo_data.right_side.elbow.motor.enabled = true;
            }
        #endif

        //Print to tell you if motors are enabled, the parameters are set, and if the functions for the first run are complete
        #ifdef MAIN_DEBUG
            #ifdef HEADLESS
                logger::print("Superloop :: Motors Enabled");
                logger::print("Superloop :: Parameters Set");
            #endif
            logger::print("Superloop :: End First Run Conditional");
        #endif
    }

    // 不使用上位机 APP 时，执行所需的校准操作     Run the calibrations we need to do if not using the app
    #ifdef HEADLESS
        
        static bool static_calibration_done = false;    //标记静态校准是否完成
        unsigned int pause_after_static_calibration_ms = 10000; //静态校准后暂停的时间（10秒）
        static unsigned int time_dynamic_calibration_finished;  //记录动态校准完成的时间
        static bool pause_between_calibration_done = false;     //标记校准间暂停是否完成
        static bool dynamic_calibration_done = false;           //标记动态校准是否完成
        
        //Data Plotting 
        static float old_time = micros();
        float new_time = micros();
        if(new_time - old_time > 10000 && dynamic_calibration_done)
        {
            //Uncomment which plots you would want in Serial Monitor, can always change what is plotting too
            #ifdef MAKE_PLOTS
                //logger::print(exo_data.left_side.hip.motor.t_ff);
                //logger::print(", ");
                //logger::print(exo_data.left_side.hip.motor.i);
                //logger::print(", ");
                //logger::print(exo_data.right_side.hip.motor.t_ff);
                //logger::print(", ");
                //logger::print(exo_data.right_side.hip.motor.i);
                //logger::print(", ");
                //logger::print(exo_data.left_side.ankle.motor.t_ff);
                //logger::print(", ");
                //logger::print(exo_data.right_side.hip.motor.i);
                //logger::print(", ");
                //logger::print(exo_data.right_side.ankle.motor.t_ff);
                //logger::print(", ");
                //logger::print(exo_data.right_side.ankle.motor.i);
                //logger::print(", ");
                //logger::print(exo_data.right_side.hip.torque_reading);
                //logger::print("\n");
            #endif

            old_time = new_time;    //new_time与old_time之间创建了一个10毫秒的定时触发条件，相当于以约100Hz的频率执行数据输出操作
            
        }
        
        // 校准扭矩传感器       Calibrate the Torque Sensors
        if ((!static_calibration_done) && (!exo_data.left_side.ankle.calibrate_torque_sensor && !exo_data.right_side.ankle.calibrate_torque_sensor))
        {
            #ifdef MAIN_DEBUG
              logger::print("Superloop : Static Calibration Done");
            #endif
            
            static_calibration_done = true;
            //记录当前时间到time_dynamic_calibration_finished变量，用于后续的时间计算
            time_dynamic_calibration_finished = millis();
            exo_data.set_status(status_defs::messages::test);
        }
    
        // 在静态校准（扭矩传感器，静止站立状态）和动态校准（FSR 足底压力传感器，行走状态）之间暂停，留出准备开始行走的时间
        // Pause between static (torque sensor, standing still) and dynamic (FSRs, during walking) calibration so we have time to start walking
        if (!pause_between_calibration_done && (static_calibration_done && ((time_dynamic_calibration_finished +  pause_after_static_calibration_ms) < millis() ))) 
        {
            #ifdef MAIN_DEBUG
              logger::print("Superloop : Pause Between Calibration Finished");
            #endif
            
            if(exo_data.left_side.is_used)
            {
                exo_data.left_side.do_calibration_toe_fsr = true;              
                exo_data.left_side.do_calibration_refinement_toe_fsr = true;   
                exo_data.left_side.do_calibration_heel_fsr = true;             
                exo_data.left_side.do_calibration_refinement_heel_fsr = true;  
            }
           
            if(exo_data.right_side.is_used)
            {
                exo_data.right_side.do_calibration_toe_fsr = true;
                exo_data.right_side.do_calibration_refinement_toe_fsr = true;
                exo_data.right_side.do_calibration_heel_fsr = true;
                exo_data.right_side.do_calibration_refinement_heel_fsr = true;
            }
            // 校准间暂停阶段已完成
            pause_between_calibration_done = true;
        }
            
        // 动态校准完成后，设置各关节的控制器When we are done with the dynamic calibrations, set the controllers
        if ((!dynamic_calibration_done) && (pause_between_calibration_done) && (!exo_data.left_side.do_calibration_toe_fsr && !exo_data.left_side.do_calibration_refinement_toe_fsr && !exo_data.left_side.do_calibration_heel_fsr && !exo_data.left_side.do_calibration_refinement_heel_fsr))
        {
            #ifdef MAIN_DEBUG
                logger::print("Superloop : Dynamic Calibration Done");
            #endif
            
            if (exo_data.left_side.hip.is_used)
            {
                // 配置各关节的默认控制器(控制器在config.ini文件中定义)     Set the default controller
                exo_data.left_side.hip.controller.controller = config_info::config_to_send[config_defs::exo_hip_default_controller_idx];
                exo.left_side.get_hip().set_controller(exo_data.left_side.hip.controller.controller);
                
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Left Hip Controller Set");
                #endif
            }
            
            if (exo_data.right_side.hip.is_used)
            {
                //Set the default controller
                exo_data.right_side.hip.controller.controller = config_info::config_to_send[config_defs::exo_hip_default_controller_idx];
                exo.right_side.get_hip().set_controller(exo_data.right_side.hip.controller.controller); 
                
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Right Hip Controller Set");
                #endif
            }

            if (exo_data.left_side.knee.is_used)
            {
                //Set the default controller
                exo_data.left_side.knee.controller.controller = config_info::config_to_send[config_defs::exo_knee_default_controller_idx];
                exo.left_side.get_knee().set_controller(exo_data.left_side.knee.controller.controller);
                
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Left Knee Controller Set");
                #endif
            }
            
            if (exo_data.right_side.knee.is_used)
            {
                //Set the default controller
                exo_data.right_side.knee.controller.controller = config_info::config_to_send[config_defs::exo_knee_default_controller_idx];
                exo.right_side.get_knee().set_controller(exo_data.right_side.knee.controller.controller); 
                
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Right Knee Controller Set");
                #endif
            }
            
            if (exo_data.left_side.ankle.is_used)
            {
                //Set the default controller
                exo_data.left_side.ankle.controller.controller = config_info::config_to_send[config_defs::exo_ankle_default_controller_idx];
                exo.left_side.get_ankle().set_controller(exo_data.left_side.ankle.controller.controller);
                
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Left Ankle Controller Set");
                #endif
            }
      
            if (exo_data.right_side.ankle.is_used)
            {
                //Set the default controller
                exo_data.right_side.ankle.controller.controller = config_info::config_to_send[config_defs::exo_ankle_default_controller_idx];
                exo.right_side.get_ankle().set_controller(exo_data.right_side.ankle.controller.controller);
                
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Right Ankle Controller Set");
                #endif
            }

            if (exo_data.left_side.elbow.is_used)
            {
                //Set the default controller
                exo_data.left_side.elbow.controller.controller = config_info::config_to_send[config_defs::exo_elbow_default_controller_idx];
                exo.left_side.get_elbow().set_controller(exo_data.left_side.elbow.controller.controller);
                
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Left Elbow Controller Set");
                #endif
            }
      
            if (exo_data.right_side.elbow.is_used)
            {
                //Set the default controller
                exo_data.right_side.elbow.controller.controller = config_info::config_to_send[config_defs::exo_elbow_default_controller_idx];
                exo.right_side.get_elbow().set_controller(exo_data.right_side.elbow.controller.controller);
                
                #ifdef MAIN_DEBUG
                    logger::print("Superloop : Right Elbow Controller Set");
                #endif
            }
            
            dynamic_calibration_done = true;
          
        }
    #endif                                                                                        

    // 运行外骨骼控制运算（如需查看函数调用流程，可查阅 exo.h/exo.cpp 文件）    Run the exo calculations (go to exo.h/exo.cpp to follow the cascade of functions this runs)
    bool ran = exo.run();     
    
    //Print some dots so we know it is doing something if we are trying to debug
    #ifdef MAIN_DEBUG
        unsigned int dot_print_ms = 5000;
        static unsigned int last_dot_time = millis();
        if(millis() - last_dot_time > dot_print_ms)
        {
          last_dot_time = millis();
          logger::print(".");
        }
    #endif 
}

//Nano Operation
#elif defined(ARDUINO_ARDUINO_NANO33BLE) | defined(ARDUINO_NANO_RP2040_CONNECT)  //Board name is ARDUINO_[build.board] property in the board.txt file found at C:\Users\[USERNAME]\AppData\Local\Arduino15\packages\arduino\hardware\mbed_nano\2.6.1  They just already prepended it with ARDUINO so you have to do it twice.

#include <stdint.h>
#include "src/ParseIni.h"
#include "src/ExoData.h"
#include "src/ComsMCU.h"
#include "src/Config.h"
#include "src/Utilities.h"

//Board to board coms
#include "src/UARTHandler.h"
#include "src/uart_commands.h"
#include "src/UART_msg_t.h"
#include "src/ComsLed.h"
#include "src/RealTimeI2C.h"
#include "src/GetBulkChar.h"

#include "src/WaistBarometer.h"
#include "src/InclineDetector.h"

#define MAIN_DEBUG 0

//Create an array to store config
namespace config_info
{
     uint8_t config_to_send[ini_config::number_of_keys] = {
            1,  //Board name
            3,  //Board version
            2,  //Battery
            22,  //Exo name
            3,  //Exo side
            5,  //Hip
            5,  //Knee
            5,  //Ankle
            4,  //Hip gear
            4,  //Knee gear
            4,  //Ankle gear
            6,  //Hip default controller
            6,  //Knee default controller
            10, //Ankle default controller
            6,  //Elbow default controller
            2,  //Hip use torque sensor
            2,  //Knee use torque sensor
            2,  //Ankle use torque sensor
            2,  //Elbow use torque sensor
            4,  //Hip flip motor dir
            4,  //Knee flip motor dir
            4,  //Ankle flip motor dir
            4,  //Elbow flip motor dir
            4,  //Hip flip torque dir
            4,  //Knee flip torque dir
            4,  //Ankle flip torque dir
            4,  //Elbow flip torque dir
            4,  //Hip flip angle dir
            4,  //Knee flip angle dir
            4,  //Ankle flip angle dir
            4,  //Elbow flip angle dir
          };
}

void setup()
{
    Serial.begin(115200);
	
	long initialTime = millis();
	readSingleMessageBlocking();
	long time_spent = millis() - initialTime;
	//delay(5000);
	Serial.print("\nNano Boot time added: ");
	Serial.print(time_spent);
	Serial.println("\n--- MESSAGE RECEIVED (Full Frame) ---");
    Serial.print("Frame Size: ");
    Serial.println(strlen(rxBuffer_bulkStr)); 
    Serial.print("Frame: ");
    Serial.println(rxBuffer_bulkStr);
	
    #if MAIN_DEBUG
      while (!Serial);
        logger::print("Setup->Getting config");
    #endif
    
    //Get the SD card config from the teensy, this has a timeout
    UARTHandler* handler = UARTHandler::get_instance();
    const bool timed_out = UART_command_utils::get_config(handler, config_info::config_to_send, (float)UART_times::CONFIG_TIMEOUT);

    //Creates new instance of LED on communication board (Nano)
    ComsLed* led = ComsLed::get_instance();

    //If there is a time out, set the LED to Yellow, otherwise turn the LED green
    if (timed_out)
    {
        #if MAIN_DEBUG
        logger::print("Setup->Timed Out Getting Config", LogLevel::Warn);
        #endif

        //Yellow
        led->set_color(255, 255, 0);
    }
    else
    {
        //Green
        led->set_color(0, 255, 0);
    }

    #if REAL_TIME_I2C
      logger::print("Init I2C");  
      real_time_i2c::init();
      logger::print("Setup->End Setup");
    #endif
}

void loop()
{
    #if MAIN_DEBUG
        
        static bool first_run = true;
        
        if (first_run)
        {
          logger::println("Start Loop");
        }
        
    #endif

    //Constructs a new ExoData object with configuration
    static ExoData* exo_data = new ExoData(config_info::config_to_send);
    
    #if MAIN_DEBUG
        if (first_run)
        {
          logger::println("Construced exo_data");
        }
    #endif

    //Constructs a new ComsMCU object with the exo data and the configuration information
    static ComsMCU* mcu = new ComsMCU(exo_data, config_info::config_to_send);
    
    #if MAIN_DEBUG
        if (first_run)
        {
          logger::println("Construced mcu");
        }
    #endif

    //Performs key communication protocols
    mcu->handle_ble();
    mcu->local_sample();
    mcu->update_UART();
    mcu->update_gui();
    mcu->handle_errors();

    #if MAIN_DEBUG
        static float then = millis();
        float now = millis();
        if ((now - then) > 1000)
        {
            then = now;
            logger::println("...");
        }
        first_run = false;
    #endif
    
}

#else //Code that operates when the microcontroller is not recognized

#include "Utilities.h"

void setup()
{
  Serial.begin(115200);
  utils::spin_on_error_with("Unknown Microcontroller");
}

void loop()
{

}

#endif
