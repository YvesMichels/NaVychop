#include "control.h"
#include "navigation.h"

struct parameters {
    ControlGains control_parameters;

};

struct states {
    ControlState control_states;
    Quaternion q_M_GRV;
    
};