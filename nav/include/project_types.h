#include "control.h"
#include "navigation.h"

struct parameters {
    ControlParameters control_param;
    
};

struct states {
    ControlState control_states;
    Quaternion q_M_GRV;
    
};