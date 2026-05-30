#include "datalink.h"

void datalink_set_param_lora(parameters &param, uint8_t lora_cmd[4]) {
    // Vérification de la commande
    if (lora_cmd[0] != LORA_CMD_SET_PARAM)
        return;

    uint8_t idx = lora_cmd[1];

    // Conversion :
    // lora_cmd[2] = partie entière
    // lora_cmd[3] = partie fractionnaire sur 8 bits
    float value = (float)lora_cmd[2] +
                  ((float)lora_cmd[3] / 255.0f);

    switch (idx)
    {
        case 0:     param.control_param.Kp_yaw = value; break;
        case 1:     param.control_param.Ki_yaw = value; break;
        case 2:     param.control_param.Kd_yaw = value; break;

        case 3:     param.control_param.Kp_pitch = value; break;
        case 4:     param.control_param.Ki_pitch = value; break;
        case 5:     param.control_param.Kd_pitch = value; break;

        case 6:     param.control_param.Kp_roll = value; break;
        case 7:     param.control_param.Ki_roll = value; break;
        case 8:     param.control_param.Kd_roll = value; break;

        case 9:     param.control_param.ctrl_bias_yaw = value; break;
        case 10:    param.control_param.ctrl_bias_pitch = value; break;
        case 11:    param.control_param.ctrl_bias_roll = value; break;

        default:    break;  // indice inconnu
           

    }
}