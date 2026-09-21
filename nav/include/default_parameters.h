#ifndef DEF_PARAM_H
#define DEF_PARAM_H

/* configuration défaut PID */
constexpr float def_kp_yaw = 0.0f;//40.0f;         // en cmd_moteur / projection_axe ([-32;31]/[-1;1])
constexpr float def_ki_yaw = 0.0f;//0.01f;        // en cmd_moteur / (projection_axe*ms) ([-32;31]/[-def_max_eta_yaw;def_max_eta_yaw])
constexpr float def_kd_yaw = 0.0f;//10.0f;         // en cmd_moteur / (projection_axe/ms) (dt*[-32;31]/[-1;1])

constexpr float def_kp_pitch = 30.0f;       // en cmd_moteur / projection_axe ([-32;31]/[-1;1])
constexpr float def_ki_pitch = 0.02f;      // en cmd_moteur / (projection_axe*ms) ([-32;31]/[-def_max_eta_pitch;def_max_eta_pitch])
constexpr float def_kd_pitch = 20.0f;       // en cmd_moteur / (projection_axe/ms) (dt*[-32;31]/[-1;1])

constexpr float def_kp_roll = 50.0f;        // en cmd_moteur / projection_axe ([-32;31]/[-1;1])
constexpr float def_ki_roll = 0.05f;       // en cmd_moteur / (projection_axe*ms) ([-32;31]/[-def_max_eta_roll;def_max_eta_roll])
constexpr float def_kd_roll = 20.0f;        // en cmd_moteur / (projection_axe/ms) (dt*[-32;31]/[-1;1])

constexpr float def_max_eta_yaw = 0.0f;//900.0f;   // 0.9 d'écart pendant une seconde
constexpr float def_max_eta_pitch = 300.0f; // 0.3 d'écart pendant une seconde
constexpr float def_max_eta_roll = 100.0f;  // 0.1 d'écart pendant une seconde

#endif