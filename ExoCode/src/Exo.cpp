/*
 * 
 * P. Stegall Jan. 2022
*/

#include "Exo.h"
#include "Time_Helper.h"
#include "UARTHandler.h"
#include "UART_msg_t.h"
#include "uart_commands.h"
#include "Logger.h"

//#define EXO_DEBUG  //Uncomment if you want the debug statements to print to serial monitor

//Arduino compiles everything in the src folder even if not included so it causes and error for the nano if this is not included.
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41) 
/*
 * Constructor for the Exo
 * Takes the exo_data
 * Uses initializer list for sides.
 * Only stores these objects, and exo_data pointer.
 * 
 * Exo 类的构造函数
 * 接收 exo_data 指针
 * 使用初始化列表初始化左右侧肢体对象
 * 仅存储这些对象以及 exo_data 指针
 */
Exo::Exo(ExoData* exo_data)
: left_side(true, exo_data)      //Constructor: uses initializer list for the sides
, right_side(false, exo_data)    //Constructor: uses initializer list for the sides
, sync_led(logic_micro_pins::sync_led_pin, sync_time::SYNC_START_STOP_HALF_PERIOD_US, sync_time::SYNC_HALF_PERIOD_US, logic_micro_pins::sync_led_on_state, logic_micro_pins::sync_default_pin)  //Create a sync LED object, the first and last arguments (pin) are found in Board.h, and the rest are in Config.h. If you do not have a digital input for the default state you can remove SYNC_DEFAULT_STATE_PIN.
, status_led(logic_micro_pins::status_led_r_pin, logic_micro_pins::status_led_g_pin, logic_micro_pins::status_led_b_pin)  //Create the status LED object.

#ifdef USE_SPEED_CHECK
    ,speed_check(logic_micro_pins::speed_check_pin)   //speed_check()函数也是构造函数初始化的对象, 在定义了USE_SPEED_CHECK宏时才会被编译
#endif

{
    //data指向传入的exo_data指针, 注入式的数据传入
    this->data = exo_data;
    //调用 ExoData 的 set_default_parameters 方法，为所有关节设置默认参数
    data->set_default_parameters();
    
    #ifdef EXO_DEBUG
        logger::println("Exo :: Constructor : _data set");
    #endif

    if (logic_micro_pins::motor_stop_available)
    {
        pinMode(logic_micro_pins::motor_stop_pin, INPUT_PULLUP);
    }
    
    #ifdef EXO_DEBUG 
        logger::println("Exo :: Constructor : motor_stop_pin Mode set");
    #endif
};

/* 
 * Run the exo 
 */
bool Exo::run()
{
    //Check if we are within the system frequency we want.
    static UARTHandler* handler = UARTHandler::get_instance();
    static Time_Helper* t_helper = Time_Helper::get_instance();
    static float context = t_helper->generate_new_context();

    // 时间差值
    static float delta_t = 0;
    static uint16_t prev_status = data->get_status();
    delta_t += t_helper->tick(context);

    //Check if the real time data is ready to be sent.
    static float rt_context = t_helper->generate_new_context();
    static float rt_delta_t = 0;

    static const float lower_bound = (float) 1/LOOP_FREQ_HZ * 1000000 * (1 - LOOP_TIME_TOLERANCE);
    
    if (delta_t >= (lower_bound))
    {    
        #if USE_SPEED_CHECK
            logger::print(String(delta_t) + "\n");
            speed_check.toggle();
        #endif

        //Check if we should update the sync LED and record the LED on/off state.
        // 判定是否需要更新同步 LED 状态，并记录 LED 的亮灭状态
        data->sync_led_state = sync_led.handler();
        bool trial_running = sync_led.get_is_blinking();

        //Check the estop
        // Emergency Stop 急停开关
        // DEMO-ONLY OVERRIDE: keep this false while the presentation setup has
        // no validated physical E-stop. Change only this value to true after
        // the switch wiring and polarity have been verified on the target PCB.
        static constexpr bool enable_physical_estop = false;

        // INPUT_PULLUP + a normally-closed stop loop: LOW permits operation;
        // an opened loop, disconnected wire, or pressed stop reads HIGH.
        const bool physical_estop_triggered = logic_micro_pins::motor_stop_available &&
            digitalRead(logic_micro_pins::motor_stop_pin) ==
                logic_micro_pins::motor_stop_active_state;
        data->estop = enable_physical_estop && physical_estop_triggered;

        // The complete E-stop shutdown path remains active for later restoration.
        if (data->estop)
        {
            data->for_each_joint([](JointData* j_data, float* args){j_data->motor.enabled = false;});
        }

        const uint16_t status_at_cycle_start = data->get_status();
        if (status_at_cycle_start == status_defs::messages::trial_on &&
            prev_status != status_defs::messages::trial_on)
        {
            // Do not reuse a phase estimate from a previous trial/calibration.
            left_side.clear_step_time_estimate();
            right_side.clear_step_time_estimate();
        }
        prev_status = status_at_cycle_start;
		
        //Remember the status that initiated an FSR calibration. Side::check_calibration
        //temporarily replaces it with fsr_calibration/fsr_refinement; restore only
        //after both sides have completed so a strict trial_on controller can restart.
        static bool fsr_calibration_session = false;
        static uint16_t status_after_fsr_calibration = status_defs::messages::trial_off;
        const bool fsr_pending_before =
            (data->left_side.is_used &&
             (data->left_side.do_calibration_toe_fsr ||
              data->left_side.do_calibration_refinement_toe_fsr ||
              data->left_side.do_calibration_heel_fsr ||
              data->left_side.do_calibration_refinement_heel_fsr)) ||
            (data->right_side.is_used &&
             (data->right_side.do_calibration_toe_fsr ||
              data->right_side.do_calibration_refinement_toe_fsr ||
              data->right_side.do_calibration_heel_fsr ||
              data->right_side.do_calibration_refinement_heel_fsr));
        const uint16_t status_before_fsr = data->get_status();
        if (fsr_pending_before && !fsr_calibration_session)
        {
            fsr_calibration_session = true;
            status_after_fsr_calibration =
                status_before_fsr == status_defs::messages::trial_on
                    ? status_defs::messages::trial_on
                    : status_defs::messages::trial_off;
        }
        else if (fsr_calibration_session &&
                 status_before_fsr == status_defs::messages::trial_off)
        {
            // A stop request received during calibration must win.
            status_after_fsr_calibration = status_defs::messages::trial_off;
        }

        //Record the side data and send new commands to the motors.
        left_side.run_side();
        right_side.run_side();

        const bool fsr_pending_after =
            (data->left_side.is_used &&
             (data->left_side.do_calibration_toe_fsr ||
              data->left_side.do_calibration_refinement_toe_fsr ||
              data->left_side.do_calibration_heel_fsr ||
              data->left_side.do_calibration_refinement_heel_fsr)) ||
            (data->right_side.is_used &&
             (data->right_side.do_calibration_toe_fsr ||
              data->right_side.do_calibration_refinement_toe_fsr ||
              data->right_side.do_calibration_heel_fsr ||
              data->right_side.do_calibration_refinement_heel_fsr));
        if (fsr_calibration_session && !fsr_pending_after)
        {
            const uint16_t current_status = data->get_status();
            if (current_status == status_defs::messages::fsr_calibration ||
                current_status == status_defs::messages::fsr_refinement)
            {
                data->set_status(status_after_fsr_calibration);
            }
            fsr_calibration_session = false;
        }
		
        //Update status LED
        status_led.update(data->get_status());
        #ifdef EXO_DEBUG
            logger::println("Exo::Run:Time_OK");
            logger::println(delta_t);
            logger::println(((float)1 / LOOP_FREQ_HZ * 1000000 * (1 + LOOP_TIME_TOLERANCE)));
        #endif

        //Check for incoming UART messages
        UART_msg_t msg = handler->poll(UART_times::CONT_MCU_TIMEOUT);       //UART_times::CONT_MCU_TIMEOUT is in Config.h
        UART_command_utils::handle_msg(handler, data, msg);

        //Send the coms mcu the real time data every _real_time_msg_delay microseconds
        rt_delta_t += t_helper->tick(rt_context);
        uint16_t exo_status = data->get_status();
        const bool correct_status = (exo_status == status_defs::messages::trial_on) || (exo_status == status_defs::messages::fsr_calibration) || (exo_status == status_defs::messages::fsr_refinement) || (exo_status == status_defs::messages::error);
        
        if ((rt_delta_t >= BLE_times::_real_time_msg_delay) && (correct_status))
        {
            #ifdef EXO_DEBUG
                logger::print("Exo::run->Sending Real Time Message: ");
                logger::println(rt_delta_t);
            #endif
            
            UART_msg_t msg;
            UART_command_handlers::get_real_time_data(handler, data, msg, data->config);
            rt_delta_t = 0;
        }

        delta_t = 0;
        return true;
    }

    return false;
};



#endif
