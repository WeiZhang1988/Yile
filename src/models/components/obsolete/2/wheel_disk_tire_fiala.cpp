#include "wheel_disk_tire_fiala.hpp"

NMSPC::Wheel_Disk_Tire_Fiala::Wheel_Disk_Tire_Fiala (\
const d_vec &params, \
const d_vec &init_states, const bool &init_locked_flag, \
const double &t) {
	//initialize parameters
	//wheel
    m_IYY = params[0];
    m_br = params[1];
    m_disk_abore = params[2];
    m_num_pads = params[3];
    m_Rm = params[4];
    m_mu_kinetic = params[5];
    m_mu_static = params[6];
    //tire
    m_unloaded_radius = params[7];
    m_Lrelx = params[8];
    m_Lrely = params[9];
    m_alpha_min = params[10];
    m_alpha_max = params[11];
	m_mu_min = params[12];
	m_mu_max = params[13];
	m_aMy = params[14];
	m_bMy = params[15];
	m_cMy = params[16];
	m_alphaMy = params[17];
	m_betaMy = params[18];
	m_Fz_min = params[19];
	m_Fz_max = params[20];
	m_cKappa = params[21];
	m_cAlpha = params[22];
	m_bMz = params[23];
	m_width = params[24];
	m_cGamma = params[25];
    //initialize continuous and discrete states
    //wheel
    m_unlocked_omega = init_states[0];
    m_unlocked_omega_pre = init_states[0];
	m_locked_flag = init_locked_flag;
	//tire
	m_kappa = init_states[1];
    m_alpha_prime = init_states[2];
	m_Mroll = init_states[3];
	//initialize time
	m_t = t;
	//feed vector of continuous states
	//wheel
	m_con_states[0] = m_unlocked_omega;
	//tire
	m_con_states[1] = m_kappa;
	m_con_states[2] = m_alpha_prime;
	m_con_states[3] = m_Mroll;
	//initialize kinematics outputs
	m_output_omega = init_states[0];
}

void NMSPC::Wheel_Disk_Tire_Fiala::preupdate (const d_vec &u, \
const double &t) {
	m_con_states[0] = m_unlocked_omega;
	
	//get system time
	m_t = t;
	
	//get inputs
    m_Brake_Pressure = u[0];
    m_Axl_trq = u[1];
    m_vx = u[2];
	m_vy = u[3];
	m_susF = u[4];
	m_gamma = u[5];
	m_psidot = u[6];
	m_tirePrs = u[7];
	m_Gnd = u[8];
	m_scale = u[9];

	//calculate middle variables
	//tire---
	m_z = m_Gnd;
    m_z_dot = 0.0;
    m_rhoz = 0;
	//--out
    m_Fz = m_susF;
    m_sat_Fz = saturation(m_Fz, m_Fz_min, m_Fz_max);
	//--
	m_Re = m_unloaded_radius - m_rhoz;
    m_alpha = saturation(m_alpha_prime * tanh(abs(m_vy)), m_alpha_min, \
    m_alpha_max);
    m_tan_alpha = tan(m_alpha);
    //--
    m_mu = (m_mu_max - (m_mu_max - m_mu_min) * saturation(sqrt(pow(\
    m_kappa, 2.0) + pow(m_tan_alpha, 2.0)), 0.0, 1.0)) * m_scale;
    //--output
    m_My = -(m_aMy + abs(m_vx) * m_bMy + \
    pow(m_vx, 2.0) * m_cMy) * \
    tanh(4.0 * m_output_omega) * m_Re * pow(m_sat_Fz, m_betaMy) * \
    pow(m_tirePrs, m_alphaMy);
    //--
    m_kappa_critical = m_mu * m_sat_Fz / 2.0 / m_cKappa;
    //--output
    if (abs(m_kappa) <= m_kappa_critical) {
    	m_Fx_stick = m_cKappa * m_kappa;
    	m_Fx = m_Fx_stick;
    } else {
    	m_Fx1 = m_mu * m_sat_Fz;
    	m_Fx2 = abs(pow(m_Fx1, 2.0) / m_cKappa / div0protect(4.0 * \
    	m_kappa, 0.01));
    	m_Fx_slide = (m_Fx1 - m_Fx2) * tanh(4.0 * m_kappa);
    	m_Fx = m_Fx_slide;
    }
    
    m_alpha_critical = atan(m_mu * 3.0 * m_sat_Fz / m_cAlpha);
    //output
    if (abs(m_alpha) <= m_alpha_critical) {
    	m_H = 1.0 - abs(m_tan_alpha) * m_cAlpha / (m_mu * 3.0 * m_sat_Fz);
    	m_Mz_stick = -m_psidot * m_bMz - m_width * (1.0 - m_H) * \
    	pow(m_H, 3.0) * m_mu * m_sat_Fz * tanh(4.0 * m_alpha);
    	m_Fy_stick = m_gamma * m_cGamma - (1.0 - pow(m_H, 3.0)) * m_mu *\
    	m_sat_Fz * tanh(4.0 * m_alpha);
    	m_Mz = m_Mz_stick;
    	m_Fy = m_Fy_stick;
    } else {
    	m_Mz_slide = m_psidot * m_bMz;
    	m_Fy_slide = -m_sat_Fz * m_mu * tanh(4.0 * m_alpha);
    	m_Mz = m_Mz_slide;
    	m_Fy = m_Fy_slide;
    }
    //--output
    m_Mx = cos(m_gamma) * m_Fy * m_Re;
    //---tire 
	//wheel---
    m_Tout = -m_Axl_trq - m_My + m_Fx * saturation(m_Re, 0.0, inf);
    m_Tfmaxk = m_Rm * m_mu_kinetic * saturation(m_Brake_Pressure * m_disk_abore * m_disk_abore * m_num_pads * pi / 4.0, eps, inf);
    m_Tfmaxs = m_Tfmaxk * m_mu_static / m_mu_kinetic;
    //update lock logic
    int logic1 = static_cast<int>((abs(m_Tout) <= m_Tfmaxs) && (m_unlocked_omega * m_unlocked_omega_pre <=0));
    int logic2 = static_cast<int>((abs(m_Tout) >= m_Tfmaxs));
    m_locked_flag = static_cast<bool>(m_truth_table[4 * logic1 + 2 * logic2 + static_cast<int>(m_locked_flag)]);
    //calculate output
    if (m_locked_flag) {
        m_output_omega = 0.0;
        m_unlocked_omega = m_output_omega;
    }
    else { 
        m_output_omega = m_unlocked_omega;
    } 
    //---wheel
}

void NMSPC::Wheel_Disk_Tire_Fiala::calculate_derivatives(double &drv_omega,\
double &drv_kappa, double &drv_alpha_prime, double &drv_Mroll) {
	//wheel
    if (m_locked_flag)
        drv_omega = 0.0;
    else
        drv_omega = (m_Tfmaxk * tanh(-4.0 * m_unlocked_omega) - m_Tout -\
        m_br * m_unlocked_omega) / m_IYY;
    //tire
    drv_kappa = (m_output_omega * m_Re - m_vx - abs(m_vx) * \
    m_kappa) / m_Lrelx;
    drv_alpha_prime = (m_vy - abs(m_vx) * m_tan_alpha) / m_Lrely; 
    //drv_Mroll = saturation(dead_zone(abs(m_vx-m_output_omega*m_Re), \
    0.0, 0.001)/saturation(m_Lrelx, 0.01, 10.0), 2.0*pi, 400.0*2.0*pi) *\
    (m_My - m_Mroll);
    drv_Mroll = ((1.0 - abs(m_vx)) * 2 * pi + abs(m_vx - \
    m_output_omega * m_Re) / saturation(m_Lrelx, 0.01, 10.0)) * \
    (m_My - m_Mroll);
}

void NMSPC::Wheel_Disk_Tire_Fiala::postupdate () {
	//wheel
	m_unlocked_omega_pre = m_unlocked_omega;
	m_unlocked_omega = m_con_states[0];
	//tire
	m_kappa = m_con_states[1];
	m_alpha_prime = m_con_states[2];
	m_Mroll = m_con_states[3];    
}

void NMSPC::Wheel_Disk_Tire_Fiala::output (d_vec &outputs) {
	//wheel
    outputs[0] = m_output_omega;
    //tire
    outputs[1] = m_Fx;
    outputs[2] = m_Fy;
    outputs[3] = m_Fz;
    outputs[4] = m_Mx;
    outputs[5] = m_Mroll;
    outputs[6] = m_Mz;
    outputs[7] = m_Re;
}
