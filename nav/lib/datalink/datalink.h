#ifndef DATALINK_H
#define DATALINK_H
#include <arduino.h>
#include "navigation.h"
#include "control.h"
#include "config.h"
#include "project_types.h"

void datalink_set_param_lora(parameters &param, uint8_t lora_cmd[4]);

#endif
