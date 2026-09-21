// *** FICHIER AUTO-GÉNÉRÉ — NE PAS MODIFIER À LA MAIN ***
// Généré par gen_datalink.py le 2026-09-21 19:47
// Source : parameters.json  (12 paramètres, 1 catégorie(s))
//
// Règle de routage protocolaire :
//   tx_type = 0x10 + (id >> 8)
//   dat0    = id & 0xFF
//   dat1    = octet bas  de la valeur int16
//   dat2    = octet haut de la valeur int16
//   valeur physique = int16 / 2^binpt  (échelle écrite en dur ci-dessous)
//
// Table des paramètres :
//    id   tx_type   dat0  binpt   scale (2^binpt)  cible C++
//  --------------------------------------------------------------------------------
//     0    0x10     0x00      4                16  system_param.control_param.Kp_yaw
//     1    0x10     0x01      8               256  system_param.control_param.Ki_yaw
//     2    0x10     0x02      4                16  system_param.control_param.Kd_yaw
//     3    0x10     0x03      4                16  system_param.control_param.Kp_pitch
//     4    0x10     0x04      8               256  system_param.control_param.Ki_pitch
//     5    0x10     0x05      4                16  system_param.control_param.Kd_pitch
//     6    0x10     0x06      4                16  system_param.control_param.Kp_roll
//     7    0x10     0x07      8               256  system_param.control_param.Ki_roll
//     8    0x10     0x08      4                16  system_param.control_param.Kd_roll
//     9    0x10     0x09      6                64  system_param.control_param.ctrl_bias_yaw
//    10    0x10     0x0A      6                64  system_param.control_param.ctrl_bias_pitch
//    11    0x10     0x0B      6                64  system_param.control_param.ctrl_bias_roll
//

#include "datalink.h"


void datalink_set_param_lora(parameters &system_param, uint8_t lora_cmd[4])
{
    // Décodage trame paramètre
    uint8_t tx_type = lora_cmd[0];
    uint8_t dat0    = lora_cmd[1];
    uint8_t dat1    = lora_cmd[2];  // LSB valeur
    uint8_t dat2    = lora_cmd[3];  // MSB valeur

    // Seules les trames de type SET_PARAM sont traitées ici.
    // tx_type vaut 0x10 pour les id 0x0000..0x00FF,
    //              0x11 pour les id 0x0100..0x01FF, etc.
    if (tx_type < 0x10)
        return;

    if (tx_type == 0x10 && dat0 == 0x00)  /* param.control_param.Kp_yaw  id=0  binpt=4 */
    {
        system_param.control_param.Kp_yaw =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 16.0f;  // 1 / 2^4
    }
    else if (tx_type == 0x10 && dat0 == 0x01)  /* param.control_param.Ki_yaw  id=1  binpt=8 */
    {
        system_param.control_param.Ki_yaw =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 256.0f;  // 1 / 2^8
    }
    else if (tx_type == 0x10 && dat0 == 0x02)  /* param.control_param.Kd_yaw  id=2  binpt=4 */
    {
        system_param.control_param.Kd_yaw =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 16.0f;  // 1 / 2^4
    }
    else if (tx_type == 0x10 && dat0 == 0x03)  /* param.control_param.Kp_pitch  id=3  binpt=4 */
    {
        system_param.control_param.Kp_pitch =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 16.0f;  // 1 / 2^4
    }
    else if (tx_type == 0x10 && dat0 == 0x04)  /* param.control_param.Ki_pitch  id=4  binpt=8 */
    {
        system_param.control_param.Ki_pitch =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 256.0f;  // 1 / 2^8
    }
    else if (tx_type == 0x10 && dat0 == 0x05)  /* param.control_param.Kd_pitch  id=5  binpt=4 */
    {
        system_param.control_param.Kd_pitch =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 16.0f;  // 1 / 2^4
    }
    else if (tx_type == 0x10 && dat0 == 0x06)  /* param.control_param.Kp_roll  id=6  binpt=4 */
    {
        system_param.control_param.Kp_roll =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 16.0f;  // 1 / 2^4
    }
    else if (tx_type == 0x10 && dat0 == 0x07)  /* param.control_param.Ki_roll  id=7  binpt=8 */
    {
        system_param.control_param.Ki_roll =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 256.0f;  // 1 / 2^8
    }
    else if (tx_type == 0x10 && dat0 == 0x08)  /* param.control_param.Kd_roll  id=8  binpt=4 */
    {
        system_param.control_param.Kd_roll =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 16.0f;  // 1 / 2^4
    }
    else if (tx_type == 0x10 && dat0 == 0x09)  /* param.control_param.ctrl_bias_yaw  id=9  binpt=6 */
    {
        system_param.control_param.ctrl_bias_yaw =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 64.0f;  // 1 / 2^6
    }
    else if (tx_type == 0x10 && dat0 == 0x0A)  /* param.control_param.ctrl_bias_pitch  id=10  binpt=6 */
    {
        system_param.control_param.ctrl_bias_pitch =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 64.0f;  // 1 / 2^6
    }
    else if (tx_type == 0x10 && dat0 == 0x0B)  /* param.control_param.ctrl_bias_roll  id=11  binpt=6 */
    {
        system_param.control_param.ctrl_bias_roll =
            (float)((int16_t)(dat1 | ((uint16_t)dat2 << 8))) / 64.0f;  // 1 / 2^6
    }

    // tx_type / dat0 inconnus : trame ignorée
}
