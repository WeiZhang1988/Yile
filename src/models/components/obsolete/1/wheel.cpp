#include "wheel.hpp"

Wheel::Wheel (const d_vec &params, const d_vec &init_states, \
const bool &init_locked_flag, const double &t) {
	//initialize parameters
    m_IYY = params[0];
    m_br = params[1];
    m_disk_abore = params[2];
    m_num_pads = params[3];
    m_Rm = params[4];
    m_mu_kinetic = params[5];
    m_mu_static = params[6];
    //initialize continuous and discrete states
    m_unlocked_omega = init_states[0];
    m_unlocked_omega_pre = init_states[0];
	m_locked_flag = init_locked_flag;
	//initialize time
	m_t = t;
	//feed vector of continuous states
	m_con_states[0] = m_unlocked_omega;
	//initialize kinematics outputs
	m_output_omega = init_states[0];
}

void Wheel::preupdate (const d_vec &u, const double &t) {
	m_con_states[0] = m_unlocked_omega;
	
	//get system time
	m_t = t;
	
	//get inputs
    m_Brake_Pressure = u[0];
    m_Axl_trq = u[1];
    m_M_y = u[2];
    m_F_x = u[3];
    m_Re = u[4];
	
	//calculate middle variables
    m_Tout = -m_Axl_trq - m_M_y + m_F_x * saturation(m_Re, 0.0, inf);
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
    
}

void Wheel::calculate_derivatives(double &drv_omega) {
    if (m_locked_flag)
        drv_omega = 0.0;
    else
        drv_omega = (m_Tfmaxk * tanh(-4.0 * m_unlocked_omega) - m_Tout - m_br * m_unlocked_omega) / m_IYY;
}

void Wheel::postupdate () {
	m_unlocked_omega_pre = m_unlocked_omega;
	m_unlocked_omega = m_con_states[0];    
}

void Wheel::output (d_vec &outputs) {
    outputs[0] = m_output_omega;
}

