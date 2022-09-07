#include "tire_fiala.hpp"

Tire_Fiala::Tire_Fiala (const d_vec &params, const d_vec &init_states, \
const double &t) {
	//initialize parameters
    m_unloaded_radius = params[0];
    m_Lrelx = params[1];
    m_Lrely = params[2];
    m_alpha_min = params[3];
    m_alpha_max = params[4];
	m_mu_min = params[5];
	m_mu_max = params[6];
	m_aMy = params[7];
	m_bMy = params[8];
	m_cMy = params[9];
	m_alphaMy = params[10];
	m_betaMy = params[11];
	m_Fz_min = params[12];
	m_Fz_max = params[13];
	m_cKappa = params[14];
	m_cAlpha = params[15];
	m_bMz = params[16];
	m_width = params[17];
	m_cGamma = params[18];
    //initialize continuous states
    m_kappa = init_states[0];
    m_alpha_prime = init_states[1];
	m_Mroll = init_states[2];
	//initialize time
	m_t = t;
	//feed vector of continuous states
	m_con_states[0] = m_kappa;
	m_con_states[1] = m_alpha_prime;
	m_con_states[2] = m_Mroll;
}

void Tire_Fiala::preupdate (const d_vec &u, const double &t) {
	m_con_states[0] = m_kappa;
	m_con_states[1] = m_alpha_prime;
	m_con_states[2] = m_Mroll;
	
	//get system time
	m_t = t;
	
	//get inputs
    m_omega = u[0];
	m_vx = u[1];
	m_vy = u[2];
	m_susF = u[3];
	m_gamma = u[4];
	m_psidot = u[5];
	m_tirePrs = u[6];
	m_Gnd = u[7];
	m_scale = u[8];
	
	//calculate middle variables and outputs
    m_z = m_Gnd;
    m_z_dot = 0.0;
    m_rhoz = 0.0;	//this is considered as output as well
    //output
    m_Fz = m_susF;
    m_sat_Fz = saturation(m_Fz, m_Fz_min, m_Fz_max);
    
    m_Re = m_unloaded_radius - m_rhoz;
    m_alpha = saturation(m_alpha_prime, m_alpha_min, \
    m_alpha_max);
    m_tan_alpha = tan(m_alpha);
    
    m_mu = (m_mu_max - (m_mu_max - m_mu_min) * saturation(sqrt(pow(\
    m_kappa, 2.0) + pow(m_tan_alpha, 2.0)), 0.0, 1.0)) * m_scale;
    
    //output
    m_My = (m_aMy + abs(m_omega) * m_bMy + pow(m_omega, 2.0) * m_cMy) * \
    tanh(4.0 * m_omega) * m_Re * pow(m_sat_Fz, m_betaMy) * \
    pow(m_tirePrs, m_alphaMy);
    
    m_kappa_critical = m_mu * m_sat_Fz / 2.0 / m_cKappa;
    //output
    if (abs(m_kappa) <= m_kappa_critical) {
    	m_Fx_stick = m_cKappa * m_kappa;
    	m_Fx = m_Fx_stick;
    } else {
    	m_Fx1 = m_mu * m_sat_Fz;
    	m_Fx2 = pow(m_Fx1, 2.0) / m_cKappa / 4.0 / m_kappa;
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
    
    //output
    m_Mx = cos(m_gamma) * m_Fy * m_Re; 
}

void Tire_Fiala::calculate_derivatives(double &drv_kappa, double &drv_alpha_prime, \
double &drv_Mroll) {
    drv_kappa = (m_omega * m_Re - m_vx - abs(m_vx) * m_kappa) / m_Lrelx;
    
    drv_alpha_prime = (m_vy - abs(m_vx) * m_tan_alpha) / m_Lrely * \
    tanh(abs(m_vy));
    
    drv_Mroll = saturation(dead_zone(abs(m_vx-m_omega*m_Re), \
    0.0, 0.001)/saturation(m_Lrelx, 0.01, 10.0), 2.0*pi, 400.0*2.0*pi) * \
    (m_My - m_Mroll);
}

void Tire_Fiala::postupdate () {
	m_kappa = m_con_states[0];
	m_alpha_prime = m_con_states[1];
	m_Mroll = m_con_states[2];

}

void Tire_Fiala::output (d_vec &outputs) {
    outputs[0] = m_Fx;
    outputs[1] = m_Fy;
    outputs[2] = m_Fz;
    outputs[3] = m_Mx;
    outputs[4] = m_Mroll;
    outputs[5] = m_Mz;
    outputs[6] = m_Re;
}

