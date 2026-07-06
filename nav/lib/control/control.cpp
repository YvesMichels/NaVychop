#include "navigation.h"
#include "control.h"
#include "config.h"
#include "default_parameters.h"

void control_init(ControlState &state)
{
    // Integrateurs 
    state.eta_yaw = 0;
    state.eta_pitch = 0;
    state.eta_roll = 0;

    // Estimateur biais lent
    state.bias_roll = 0;

    // Valeurs précédentes
    state.old_yaw = 0;
    state.old_pitch = 0;
    state.old_roll = 0;

    state.ctrl_yaw = 0;
    state.ctrl_pitch = 0;
    state.ctrl_roll = 0;

    state.init = 1;
}

void control_gain_init(ControlParameters &gains)
{
    gains.Kp_yaw = def_kp_yaw;
    gains.Ki_yaw = def_ki_yaw;
    gains.Kd_yaw = def_kd_yaw;

    gains.Kp_pitch = def_kp_pitch;
    gains.Ki_pitch = def_ki_pitch;
    gains.Kd_pitch = def_kd_pitch;

    gains.Kp_roll = def_kp_roll;
    gains.Ki_roll = def_ki_roll;
    gains.Kd_roll = def_kd_roll;

    gains.ctrl_bias_yaw = 0.0f;     // biais de position de la gouverne sur le yaw : calibration manuelle
    gains.ctrl_bias_pitch = 0.0f;   // biais de position de la gouverne sur le pitch : calibration manuelle
    gains.ctrl_bias_roll = 0.0f;    // biais de position de la gouverne sur le roll : calibration manuelle
}

inline void bound_integrator(float &value, float max_value)
{
    if(value > max_value)
    {
        value = max_value;
    }
    else if(value < -max_value)
    {
        value = -max_value;
    }
}

void control_update_inner(ControlState &state, ControlParameters &gains, float R_A_T[3][3], float cmd_yaw, float cmd_pitch, float cmd_roll)
{
    float proj_yaw;
    float proj_pitch;
    float proj_roll;

    proj_yaw = computeYawCommandSignalImproved(R_A_T, cmd_yaw);  
    proj_pitch = computePitchCommandSignal(R_A_T, cmd_pitch); 

    //cmd_roll = -0*proj_yaw;            // le roll est proportionnel à l'écart de yaw
    bound_integrator(cmd_roll, 0.4);// le roll est borné à 0.4 rad
    proj_roll = computeRollCommandSignal(R_A_T, cmd_roll);

    /*Serial.print(R_A_T[0][0]); 
    Serial.print(" - ");
    Serial.print(R_A_T[0][1]); 
    Serial.print(" - ");
    Serial.println(R_A_T[0][2]); */
    state.eta_yaw += proj_yaw * d_t;
    bound_integrator(state.eta_yaw, def_max_eta_yaw);

    state.eta_pitch += proj_pitch * d_t;
    bound_integrator(state.eta_pitch, def_max_eta_pitch);

    state.eta_roll += proj_roll * d_t;
    bound_integrator(state.eta_roll, def_max_eta_roll);


    // controle PI
    state.ctrl_yaw = proj_yaw*gains.Kp_yaw + state.eta_yaw*gains.Ki_yaw + gains.ctrl_bias_yaw;
    state.ctrl_pitch = proj_pitch*gains.Kp_pitch + state.eta_pitch*gains.Ki_pitch + gains.ctrl_bias_pitch;
    state.ctrl_roll = proj_roll*gains.Kp_roll + state.eta_roll*gains.Ki_roll + gains.ctrl_bias_roll;

    // control dérivée si init passée
    if(state.init == 0){
       state.ctrl_yaw += (proj_yaw - state.old_yaw)*gains.Kd_yaw/d_t;
       state.ctrl_pitch += (proj_pitch - state.old_pitch)*gains.Kd_pitch/d_t;
       state.ctrl_roll += (proj_roll - state.old_roll)*gains.Kd_roll/d_t;
    } 
    else{
        state.init = 0;
    }

    // mise à jour pour la prochaine dérivée
    state.old_yaw = proj_yaw;
    state.old_pitch = proj_pitch;
    state.old_roll = proj_roll;

}
