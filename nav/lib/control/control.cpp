struct ControlGains {
    float Kp_phi;
    float Kd_phi;

    float Kp_theta;
    float Kd_theta;

    float Kp_psi_ext;
    float Ki_psi_ext;

    float Kp_h_ext;
    float Ki_h_ext;

    float bias_adapt_rate;   // epsilon
};

struct ControlState {
    // Integrateurs externes
    float eta_psi;
    float eta_h;

    // Estimateur biais lent
    float bias_phi;
    float bias_theta;

    // Valeurs précédentes
    float x_prev[3];
};

void control_init(ControlState &state)
{
    state.eta_psi = 0.0f;
    state.eta_h   = 0.0f;

    state.bias_phi   = 0.0f;
    state.bias_theta = 0.0f;

    state.x_prev[0] = 1.0f;
    state.x_prev[1] = 0.0f;
    state.x_prev[2] = 0.0f;
}

void control_update_outer(
    ControlState &state,
    const ControlGains &gains,
    float e_psi,
    float e_h,
    float dt_ext,
    float &s_phi_ref,
    float &s_theta_ref)
{
    // Intégrateurs navigation
    state.eta_psi += e_psi * dt_ext;
    state.eta_h   += e_h   * dt_ext;

    s_phi_ref =
        gains.Kp_psi_ext * e_psi +
        gains.Ki_psi_ext * state.eta_psi;

    s_theta_ref =
        gains.Kp_h_ext * e_h +
        gains.Ki_h_ext * state.eta_h;
}

void control_update_inner(
    ControlState &state,
    const ControlGains &gains,
    const float R[3][3],      // matrice rotation
    float p,                  // gyro x
    float q,                  // gyro y
    float s_phi_ref,
    float s_theta_ref,
    float dt,
    float &delta_a,
    float &delta_e)
{
    // z_t = [0 0 1]
    float s_theta = R[2][0];  // x · z_t
    float s_phi   = R[2][1];  // y · z_t

    float e_phi   = s_phi_ref   - s_phi;
    float e_theta = s_theta_ref - s_theta;

    // Loi PD + biais
    delta_a =
        gains.Kp_phi * e_phi
        - gains.Kd_phi * p
        + state.bias_phi;

    delta_e =
        gains.Kp_theta * e_theta
        - gains.Kd_theta * q
        + state.bias_theta;

    // Saturation (exemple)
    const float max_cmd = 1.0f;

    bool sat_phi = false;
    bool sat_theta = false;

    if (delta_a > max_cmd) {
        delta_a = max_cmd;
        sat_phi = true;
    }
    if (delta_a < -max_cmd) {
        delta_a = -max_cmd;
        sat_phi = true;
    }

    if (delta_e > max_cmd) {
        delta_e = max_cmd;
        sat_theta = true;
    }
    if (delta_e < -max_cmd) {
        delta_e = -max_cmd;
        sat_theta = true;
    }

    // Adaptation biais lente (anti-windup)
    if (!sat_phi) {
        state.bias_phi += gains.bias_adapt_rate * e_phi * dt;
    }

    if (!sat_theta) {
        state.bias_theta += gains.bias_adapt_rate * e_theta * dt;
    }
}

float estimate_yaw_rate(
    ControlState &state,
    const float R[3][3],
    float dt)
{
    float x_curr[3] = { R[0][0], R[1][0], R[2][0] };

    float cross_z =
        state.x_prev[0] * x_curr[1] -
        state.x_prev[1] * x_curr[0];

    float yaw_rate = cross_z / dt;

    // sauvegarde
    state.x_prev[0] = x_curr[0];
    state.x_prev[1] = x_curr[1];
    state.x_prev[2] = x_curr[2];

    return yaw_rate;
}