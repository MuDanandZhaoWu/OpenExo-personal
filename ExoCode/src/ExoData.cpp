#include "ExoData.h"
#include "error_codes.h"
#include "Logger.h"
#include "ParamsFromSD.h"
#include "Config.h"

/*
外骨骼数据类的构造函数。
接收来自 INI 配置解析器的数组。
存储外骨骼状态与同步 LED 状态。
使用初始化列表初始化左右侧数据。

构造函数初始化left_side与right_side两个成员, 
即调用 SideData 的构造函数去初始化 left_side 这个对象
right_side和left_side是SideData对象
*/
ExoData::ExoData(uint8_t* config_to_send) 
: left_side(true, config_to_send)            //Using initializer list for member objects.
, right_side(false, config_to_send)
{
    this->_status = status_defs::messages::trial_off;
    this->sync_led_state = false;
    this->estop = false;

    this->config = config_to_send;
    this->config_len = ini_config::number_of_keys;

    this->mark = 10;  

    this->error_code = static_cast<int>(NO_ERROR);
    this->error_joint_id = 0;
    this->user_paused = false;
    this->start_request_pending = false;
    this->stop_request_pending = false;
    this->start_request_after_stop = false;
    this->start_fsr_calibration_sent = false;
    this->status_request_token = 0;
    this->pending_start_token = 0;
    this->pending_stop_token = 0;

    //If statement that determines if torque sensor is used for that joint (See Board.h for available torque sensor pins)
    if ((config_to_send[config_defs::hip_use_torque_sensor_idx] == (uint8_t)config_defs::use_torque_sensor::yes))
    {
        hip_torque_flag = 1;
    }

    if ((config_to_send[config_defs::knee_use_torque_sensor_idx] == (uint8_t)config_defs::use_torque_sensor::yes))
    {
        knee_torque_flag = 1;
    }

    if ((config_to_send[config_defs::ankle_use_torque_sensor_idx] == (uint8_t)config_defs::use_torque_sensor::yes))
    {
        ankle_torque_flag = 1;
    }

    if ((config_to_send[config_defs::elbow_use_torque_sensor_idx] == (uint8_t)config_defs::use_torque_sensor::yes))
    {
        elbow_torque_flag = 1;
    }

    if ((config_to_send[config_defs::arm_1_use_torque_sensor_idx] == (uint8_t)config_defs::use_torque_sensor::yes))
    {
        arm_1_torque_flag = 1;
    }

    if ((config_to_send[config_defs::arm_2_use_torque_sensor_idx] == (uint8_t)config_defs::use_torque_sensor::yes))
    {
        arm_2_torque_flag = 1;
    }
};

void ExoData::reconfigure(uint8_t* config_to_send) 
{
    start_request_pending = false;
    stop_request_pending = false;
    start_request_after_stop = false;
    start_fsr_calibration_sent = false;
    status_request_token = 0;
    pending_start_token = 0;
    pending_stop_token = 0;
    left_side.reconfigure(config_to_send);
    right_side.reconfigure(config_to_send);
};

uint8_t ExoData::get_used_joints(uint8_t* used_joints)
{
    uint8_t len = 0;

    used_joints[len] = ((left_side.hip.is_used) ? (1) : (0));
    len += left_side.hip.is_used;
    used_joints[len] = ((left_side.knee.is_used) ? (1) : (0));
    len += left_side.knee.is_used;
    used_joints[len] = ((left_side.ankle.is_used) ? (1) : (0));
    len += left_side.ankle.is_used;
    used_joints[len] = ((left_side.elbow.is_used) ? (1) : (0));
    len += left_side.elbow.is_used;
    used_joints[len] = ((left_side.arm_1.is_used) ? (1) : (0));
    len += left_side.arm_1.is_used;
    used_joints[len] = ((left_side.arm_2.is_used) ? (1) : (0));
    len += left_side.arm_2.is_used;
    used_joints[len] = ((right_side.hip.is_used) ? (1) : (0));
    len += right_side.hip.is_used;
    used_joints[len] = ((right_side.knee.is_used) ? (1) : (0));
    len += right_side.knee.is_used;
    used_joints[len] = ((right_side.ankle.is_used) ? (1) : (0));
    len += right_side.ankle.is_used;
    used_joints[len] = ((right_side.elbow.is_used) ? (1) : (0));
    len += right_side.elbow.is_used;
    used_joints[len] = ((right_side.arm_1.is_used) ? (1) : (0));
    len += right_side.arm_1.is_used;
    used_joints[len] = ((right_side.arm_2.is_used) ? (1) : (0));
    len += right_side.arm_2.is_used;
    return len;
};

JointData* ExoData::get_joint_with(uint8_t id)
{
    JointData* j_data = NULL;
    switch (id)
    {
    case (uint8_t)config_defs::joint_id::left_hip:  //只是属于switch_case语句的冒号...
        j_data = &left_side.hip;
        break;
    case (uint8_t)config_defs::joint_id::left_knee:
        j_data = &left_side.knee;
        break;
    case (uint8_t)config_defs::joint_id::left_ankle:
        j_data = &left_side.ankle;
        break;
    case (uint8_t)config_defs::joint_id::left_elbow:
        j_data = &left_side.elbow;
        break;
    case (uint8_t)config_defs::joint_id::left_arm_1:
        j_data = &left_side.arm_1;
        break;
    case (uint8_t)config_defs::joint_id::left_arm_2:
        j_data = &left_side.arm_2;
        break;
    case (uint8_t)config_defs::joint_id::right_hip:
        j_data = &right_side.hip;
        break;
    case (uint8_t)config_defs::joint_id::right_knee:
        j_data = &right_side.knee;
        break;
    case (uint8_t)config_defs::joint_id::right_ankle:
        j_data = &right_side.ankle;
        break; 
    case (uint8_t)config_defs::joint_id::right_elbow:
        j_data = &right_side.elbow;
        break;
    case (uint8_t)config_defs::joint_id::right_arm_1:
        j_data = &right_side.arm_1;
        break;
    case (uint8_t)config_defs::joint_id::right_arm_2:
        j_data = &right_side.arm_2;
        break;
    default:
        // logger::print("ExoData::get_joint_with->No joint with ");
        // logger::print(id);
        // logger::println(" was found.");
        break;
    }
    return j_data;
};

//用于设置外骨骼系统的状态
void ExoData::set_status(uint16_t status_to_set)
{
    //If the status is already error, don't change it
    if (this->_status == status_defs::messages::error)
    {
        return;
    }
    this->_status = status_to_set;
}

uint16_t ExoData::get_status(void)
{
    return this->_status;
}

bool ExoData::set_default_parameters()
{
    bool success = true;
#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
    this->for_each_joint([this, &success](JointData* j_data, float* args)
        {
            if (j_data->is_used)
            {
                const uint8_t error = set_controller_params(
                    (uint8_t)j_data->id,
                    j_data->controller.controller, 0, this);
                if (error != 0)
                {
                    print_param_error_message(error);
                    success = false;
                }
            }
        }
    );
#endif
    return success;
}

bool ExoData::set_default_parameters(uint8_t id)
{
    bool success = true;
    #if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
    float f_id = static_cast<float>(id);
    this->for_each_joint(
        [this, &success](JointData* j_data, float* args)
        {
            if (j_data->is_used && (uint8_t)j_data->id == static_cast<uint8_t>(args[0]))
            {
                const uint8_t error = set_controller_params(
                    (uint8_t)j_data->id,
                    j_data->controller.controller, 0, this);
                if (error != 0)
                {
                    print_param_error_message(error);
                    success = false;
                }
            }
        },
        &f_id   //for_each_joint函数的两个参数,一个是这里的lambda表达式, 一个是&f_id
    );          //通过泛型回调args被替换为了&f_id(参考for_each_joint在ExoData.h中的声明)
    #endif
    return success;
}

void ExoData::start_pretrial_cal()
{
    //Calibrate the Torque Sensors
    this->for_each_joint([](JointData* j_data, float* args) {j_data->calibrate_torque_sensor = j_data->is_used;});
}

float ExoData::get_batt_info(uint8_t batt_info_type)
{
	#if defined(ARDUINO_TEENSY36)  || defined(ARDUINO_TEENSY41)
	static bool ina_first_run = true;
	if (ina_first_run) {
		#if	BATTERY_SENSOR == 260
			if (!ina260.begin()) {
				Serial.println("Couldn't find INA260 chip");
				while (1);
			}
		#elif BATTERY_SENSOR == 219
			if (!ina219.begin()) {
				Serial.println("Failed to find INA219 chip");
				while (1) { delay(10); }
			}
		#elif BATTERY_SENSOR == 3
		pinMode(logic_micro_pins::volt_sense, INPUT);
		#endif
		ina_first_run = false;
	}
		#if	BATTERY_SENSOR == 260
			switch (batt_info_type)
			{
				case 0:
				float battery_voltage = 0.001 * ina260.readBusVoltage();
				if (battery_voltage < CRITICAL_BATT_VAL) {
					return -1;
				}
				return battery_voltage;
				break;
				case 1:
				{
				float battery_power = 0.001 * ina260.readPower();
				filtered_batt_pwr = utils::ewma(battery_power, filtered_batt_pwr, 0.05);
				//Serial.print("\n&&&&&&&&&&&&&*****filtered_batt_pwr: ");
				//Serial.print(filtered_batt_pwr);
				return filtered_batt_pwr;
				break;
				}
				default:
				battery_voltage = 0.001 * ina260.readBusVoltage();
				if (battery_voltage < CRITICAL_BATT_VAL) {
					return -1;
				}
				return battery_voltage;
			}
		#elif BATTERY_SENSOR == 219
			switch (batt_info_type)
			{
				case 0:
				float battery_voltage = ina219.getBusVoltage_V();
				if (battery_voltage < CRITICAL_BATT_VAL) {
					return -1;
				}
				return battery_voltage;
				break;
				case 1:
				float battery_power = 0.001 * ina219.getPower_mW();
				return battery_power;
				break;
				default:
				battery_voltage = ina219.getBusVoltage_V();
				if (battery_voltage < CRITICAL_BATT_VAL) {
					return -1;
				}
				return battery_voltage;
			}
		#elif BATTERY_SENSOR == 3
			float _sense_pin_volt = 3.3 * analogRead(logic_micro_pins::volt_sense) / 4095;
			float battery_voltage = _sense_pin_volt * (RESISTOR_1 + RESISTOR_2) / RESISTOR_2;
			return battery_voltage;
		#else
			return 0;
		#endif
	#endif
}

void ExoData::print()
{
    logger::print("\t Status : ");
    logger::println(_status);
    logger::print("\t Sync LED : ");
    logger::println(sync_led_state);
    
    if (left_side.is_used)
    {
        logger::print("\tLeft :: FSR Calibration : ");
        logger::print(left_side.do_calibration_heel_fsr);
        logger::println(left_side.do_calibration_toe_fsr);
        logger::print("\tLeft :: FSR Refinement : ");
        logger::print(left_side.do_calibration_refinement_heel_fsr);
        logger::println(left_side.do_calibration_refinement_toe_fsr);
        logger::print("\tLeft :: Percent Gait : ");
        logger::println(left_side.percent_gait);
        logger::print("\tLeft :: Heel FSR : ");
        logger::println(left_side.heel_fsr);
        logger::print("\tLeft :: Toe FSR : ");
        logger::println(left_side.toe_fsr);
        
        if(left_side.hip.is_used)
        {
            logger::println("\tLeft :: Hip");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_side.hip.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_side.hip.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_side.hip.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_side.hip.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_side.hip.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_side.hip.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_side.hip.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_side.hip.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_side.hip.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_side.hip.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_side.hip.controller.parameter_set);
        }
        
        if(left_side.knee.is_used)
        {
            logger::println("\tLeft :: Knee");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_side.knee.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_side.knee.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_side.knee.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_side.knee.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_side.knee.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_side.knee.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_side.knee.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_side.knee.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_side.knee.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_side.knee.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_side.knee.controller.parameter_set);
        }
        if(left_side.ankle.is_used)
        {
            logger::println("\tLeft :: Ankle");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_side.ankle.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_side.ankle.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_side.ankle.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_side.ankle.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_side.ankle.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_side.ankle.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_side.ankle.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_side.ankle.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_side.ankle.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_side.ankle.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_side.ankle.controller.parameter_set);
        }

        if (left_side.elbow.is_used)
        {
            logger::println("\tLeft :: Elbow");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(left_side.elbow.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(left_side.elbow.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(left_side.elbow.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(left_side.elbow.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(left_side.elbow.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(left_side.elbow.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(left_side.elbow.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(left_side.elbow.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(left_side.elbow.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(left_side.elbow.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(left_side.elbow.controller.parameter_set);
        }
    }
    
    if (right_side.is_used)
    {
        logger::print("\tRight :: FSR Calibration : ");
        logger::print(right_side.do_calibration_heel_fsr);
        logger::println(right_side.do_calibration_toe_fsr);
        logger::print("\tRight :: FSR Refinement : ");
        logger::print(right_side.do_calibration_refinement_heel_fsr);
        logger::println(right_side.do_calibration_refinement_toe_fsr);
        logger::print("\tRight :: Percent Gait : ");
        logger::println(right_side.percent_gait);
        logger::print("\tLeft :: Heel FSR : ");
        logger::println(right_side.heel_fsr);
        logger::print("\tLeft :: Toe FSR : ");
        logger::println(right_side.toe_fsr);
        
        if(right_side.hip.is_used)
        {
            logger::println("\tRight :: Hip");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_side.hip.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_side.hip.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_side.hip.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_side.hip.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_side.hip.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_side.hip.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_side.hip.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_side.hip.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_side.hip.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_side.hip.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_side.hip.controller.parameter_set);
            
        }
        
        if(right_side.knee.is_used)
        {
            logger::println("\tRight :: Knee");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_side.knee.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_side.knee.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_side.knee.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_side.knee.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_side.knee.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_side.knee.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_side.knee.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_side.knee.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_side.knee.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_side.knee.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_side.knee.controller.parameter_set);
        }
        
        if(right_side.ankle.is_used)
        {
            logger::println("\tRight :: Ankle");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_side.ankle.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_side.ankle.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_side.ankle.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_side.ankle.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_side.ankle.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_side.ankle.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_side.ankle.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_side.ankle.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_side.ankle.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_side.ankle.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_side.ankle.controller.parameter_set);
        }

        if (right_side.elbow.is_used)
        {
            logger::println("\tRight :: Elbow");
            logger::print("\t\tcalibrate_torque_sensor : ");
            logger::println(right_side.elbow.calibrate_torque_sensor);
            logger::print("\t\ttorque_reading : ");
            logger::println(right_side.elbow.torque_reading);
            logger::print("\t\tMotor :: p : ");
            logger::println(right_side.elbow.motor.p);
            logger::print("\t\tMotor :: v : ");
            logger::println(right_side.elbow.motor.v);
            logger::print("\t\tMotor :: i : ");
            logger::println(right_side.elbow.motor.i);
            logger::print("\t\tMotor :: p_des : ");
            logger::println(right_side.elbow.motor.p_des);
            logger::print("\t\tMotor :: v_des : ");
            logger::println(right_side.elbow.motor.v_des);
            logger::print("\t\tMotor :: t_ff : ");
            logger::println(right_side.elbow.motor.t_ff);
            logger::print("\t\tController :: controller : ");
            logger::println(right_side.elbow.controller.controller);
            logger::print("\t\tController :: setpoint : ");
            logger::println(right_side.elbow.controller.setpoint);
            logger::print("\t\tController :: parameter_set : ");
            logger::println(right_side.elbow.controller.parameter_set);
        }
    }
   
};
