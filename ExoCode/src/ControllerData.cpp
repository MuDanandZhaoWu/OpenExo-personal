/*
 * 
 * P. Stegall Jan. 2022
*/

#include "ControllerData.h"

/*
 * Constructor for the controller data.
 * Takes the joint id and the array from the INI parser.
 * Stores the id, sets the controller to the default controller for the appropriate joint, and records the joint type to check we are using appropriate controllers. 
 */
ControllerData::ControllerData(config_defs::joint_id id, uint8_t* config_to_send)
{
    
    switch ((uint8_t)id & (~(uint8_t)config_defs::joint_id::left & ~(uint8_t)config_defs::joint_id::right))  //Use the id with the side masked out.
    {
        case (uint8_t)config_defs::joint_id::hip:
        {
            controller = config_to_send[config_defs::exo_hip_default_controller_idx];
            joint = config_defs::JointType::hip;
            break;
        }
        case (uint8_t)config_defs::joint_id::knee:
        {
            controller = config_to_send[config_defs::exo_knee_default_controller_idx];
            joint = config_defs::JointType::knee;
            break;
        }
        case (uint8_t)config_defs::joint_id::ankle:
        {
            controller = config_to_send[config_defs::exo_ankle_default_controller_idx];
            joint = config_defs::JointType::ankle;
            break;
        }
        case (uint8_t)config_defs::joint_id::elbow:
        {
            controller = config_to_send[config_defs::exo_elbow_default_controller_idx];
            joint = config_defs::JointType::elbow;
            break;
        }
        case (uint8_t)config_defs::joint_id::arm_1:
        {
            controller = config_to_send[config_defs::exo_arm_1_default_controller_idx];
            joint = config_defs::JointType::arm_1;
            break;
        }
        case (uint8_t)config_defs::joint_id::arm_2:
        {
            controller = config_to_send[config_defs::exo_arm_2_default_controller_idx];
            joint = config_defs::JointType::arm_2;
            break;
        }
    }
    
    setpoint = 0;
    parameter_set = 0;

    for (int i=0; i < controller_defs::max_parameters; i++)
    {    
        parameters[i] = 0;
    }

    filtered_cmd = 0;
    filtered_torque_reading = 0;
};

void ControllerData::reconfigure(uint8_t* config_to_send) 
{
    //Just reset controller
    switch ((uint8_t)joint)
    {
        case (uint8_t)config_defs::JointType::hip:
        {
            controller = config_to_send[config_defs::exo_hip_default_controller_idx];
            break;
        }
        case (uint8_t)config_defs::JointType::knee:
        {
            controller = config_to_send[config_defs::exo_knee_default_controller_idx];
            break;
        }
        case (uint8_t)config_defs::JointType::ankle:
        {
            controller = config_to_send[config_defs::exo_ankle_default_controller_idx];
            break;
        }
        case (uint8_t)config_defs::JointType::elbow:
        {
            controller = config_to_send[config_defs::exo_elbow_default_controller_idx];
            break;
        }
        case (uint8_t)config_defs::JointType::arm_1:
        {
            controller = config_to_send[config_defs::exo_arm_1_default_controller_idx];
            break;
        }
        case (uint8_t)config_defs::JointType::arm_2:
        {
            controller = config_to_send[config_defs::exo_arm_2_default_controller_idx];
            break;
        }
    }
    
    setpoint = 0;

    for (int i=0; i < controller_defs::max_parameters; i++)
    {    
        parameters[i] = 0;
    }
};


uint8_t ControllerData::get_parameter_length()
{
    return get_parameter_length(controller);
}

uint8_t ControllerData::get_parameter_length(uint8_t controller_id)
{
    switch (joint)
    {
        case config_defs::JointType::hip:
            switch (controller_id)
            {
                case (uint8_t)config_defs::hip_controllers::disabled:
                case (uint8_t)config_defs::hip_controllers::zero_torque:
                    return controller_defs::zero_torque::num_parameter;
                case (uint8_t)config_defs::hip_controllers::franks_collins_hip:
                    return controller_defs::franks_collins_hip::num_parameter;
                case (uint8_t)config_defs::hip_controllers::constant_torque:
                    return controller_defs::constant_torque::num_parameter;
                case (uint8_t)config_defs::hip_controllers::chirp:
                    return controller_defs::chirp::num_parameter;
                case (uint8_t)config_defs::hip_controllers::step:
                    return controller_defs::step::num_parameter;
                case (uint8_t)config_defs::hip_controllers::phmc:
                    return controller_defs::proportional_hip_moment::num_parameter;
                case (uint8_t)config_defs::hip_controllers::calibr_manager:
                    return controller_defs::calibr_manager::num_parameter;
                case (uint8_t)config_defs::hip_controllers::spline:
                    return controller_defs::spline::num_parameter;
                case (uint8_t)config_defs::hip_controllers::fsr_hip_pd:
                    return controller_defs::fsr_hip_pd::num_parameter;
                default:
                    return 0;
            }

        case config_defs::JointType::knee:
            switch (controller_id)
            {
                case (uint8_t)config_defs::knee_controllers::disabled:
                case (uint8_t)config_defs::knee_controllers::zero_torque:
                    return controller_defs::zero_torque::num_parameter;
                case (uint8_t)config_defs::knee_controllers::constant_torque:
                    return controller_defs::constant_torque::num_parameter;
                case (uint8_t)config_defs::knee_controllers::chirp:
                    return controller_defs::chirp::num_parameter;
                case (uint8_t)config_defs::knee_controllers::step:
                    return controller_defs::step::num_parameter;
                case (uint8_t)config_defs::knee_controllers::calibr_manager:
                    return controller_defs::calibr_manager::num_parameter;
                default:
                    return 0;
            }

        case config_defs::JointType::ankle:
            switch (controller_id)
            {
                case (uint8_t)config_defs::ankle_controllers::disabled:
                case (uint8_t)config_defs::ankle_controllers::zero_torque:
                    return controller_defs::zero_torque::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::pjmc:
                    return controller_defs::proportional_joint_moment::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::zhang_collins:
                    return controller_defs::zhang_collins::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::constant_torque:
                    return controller_defs::constant_torque::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::trec:
                    return controller_defs::trec::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::calibr_manager:
                    return controller_defs::calibr_manager::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::chirp:
                    return controller_defs::chirp::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::step:
                    return controller_defs::step::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::spv2:
                    return controller_defs::spv2::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::pjmc_plus:
                    return controller_defs::pjmc_plus::num_parameter;
                case (uint8_t)config_defs::ankle_controllers::spline:
                    return controller_defs::spline::num_parameter;
                default:
                    return 0;
            }

        case config_defs::JointType::elbow:
            switch (controller_id)
            {
                case (uint8_t)config_defs::elbow_controllers::disabled:
                case (uint8_t)config_defs::elbow_controllers::zero_torque:
                    return controller_defs::zero_torque::num_parameter;
                case (uint8_t)config_defs::elbow_controllers::elbow_min_max:
                    return controller_defs::elbow_min_max::num_parameter;
                case (uint8_t)config_defs::elbow_controllers::calibr_manager:
                    return controller_defs::calibr_manager::num_parameter;
                case (uint8_t)config_defs::elbow_controllers::chirp:
                    return controller_defs::chirp::num_parameter;
                case (uint8_t)config_defs::elbow_controllers::step:
                    return controller_defs::step::num_parameter;
                default:
                    return 0;
            }

        case config_defs::JointType::arm_1:
        case config_defs::JointType::arm_2:
            if (controller_id == 1 || controller_id == 2)
            {
                return controller_defs::zero_torque::num_parameter;
            }
            if (controller_id == 3)
            {
                return controller_defs::constant_torque::num_parameter;
            }
            if (controller_id == 4)
            {
                return controller_defs::spline::num_parameter;
            }
            return 0;

        default:
            return 0;
    }
}
