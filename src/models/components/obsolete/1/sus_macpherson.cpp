#include "sus_macpherson.hpp"

Sus_Macpherson::Sus_Macpherson(const d_vec &params, const bool is_strg,\
const double &t){
	m_left_or_right   = params[0];				
	m_F0z      		  = params[1];
	m_Kz 	   		  = params[2];
	m_Cz 	   		  = params[3]; 
	m_Hmax  		  = params[4]; 		
	m_roll_strg_H_slp = params[5]; 	
	m_toe 		 	  = params[6];
	m_toe_strg_slp 	  = params[7]; 		
	m_caster 		  = params[8]; 
	m_caster_H_slp	  = params[9];
	m_caster_strg_slp = params[10];
	m_camber 		  = params[11]; 
	m_camber_H_slp    = params[12]; 
	m_camber_strg_slp = params[13]; 
	m_strg_hgt_slp    = params[14];
	m_is_strg  		  = is_strg; 
	
	m_t = t;
	
	if (!is_strg)
		m_strg_hgt_slp = 0.0;
}

void Sus_Macpherson::preupdate(const d_vec &u, const double &t){
	//get system time
	m_t = t;
	//get inputs
	m_whl_Pz   = u[0];
	m_whl_Re   = u[1];
	m_whl_Vz   = u[2];
	m_whl_Fx   = u[3];
	m_whl_Fy   = u[4];
	m_whl_Mx   = u[5];
	m_whl_My   = u[6];
	m_whl_Mz   = u[7];
	m_veh_Pz   = u[8];
	m_veh_Vx   = u[9];
	m_veh_Vy   = u[10];
	m_veh_Vz   = u[11];
	m_strg_ang = u[12];
	// middle variables and total effort
	m_veh_h = -(abs(m_strg_ang) * m_strg_hgt_slp - m_whl_Pz + m_veh_Pz);
	m_sus_h = -(m_F0z / m_Kz - m_veh_h);
	m_x_minus_hmax = -m_sus_h - m_Hmax;
	m_x_plus_hmax  = -m_sus_h + m_Hmax;
	m_x_dot 	   = m_veh_Vz - m_whl_Vz;
	//measure whether hit hard stop point
	if ((m_x_minus_hmax) > 0.0) {
		// upper hard stop
		m_hard_stop_force = calculate_hard_stop_force(m_x_minus_hmax, \
		 m_x_dot);
	} else if ((m_x_plus_hmax) < 0.0) {
		// lower hard stop
		m_hard_stop_force = calculate_hard_stop_force(m_x_plus_hmax, \
		 m_x_dot);
	} else {
		m_hard_stop_force = 0.0;
	}
	m_total_effort = (-m_sus_h * m_Kz) + ((m_veh_Vz - m_whl_Vz)*m_Cz) + \
	m_hard_stop_force;
	// wheel angles
	if (m_left_or_right <= 0.0)
		m_adjusted_toe = -(abs(m_strg_ang) * m_toe_strg_slp + m_toe + 
		m_roll_strg_H_slp * m_sus_h);
	else 
		m_adjusted_toe = abs(m_strg_ang) * m_toe_strg_slp + m_toe + \
		m_roll_strg_H_slp * m_sus_h;
	//outputs
	m_veh_Fz = -m_total_effort;
	m_whl_Fz = m_total_effort;
	//---
	m_whl_strg = m_adjusted_toe - m_toe + m_strg_ang;
	m_whl_camber = abs(m_strg_ang) * m_camber_strg_slp + m_camber + \
	m_camber_H_slp * m_sus_h;
	m_whl_caster = abs(m_strg_ang) * m_caster_strg_slp + m_caster + \
	m_caster_H_slp * m_sus_h;
	m_arm = m_veh_h + m_whl_Re;
	//---
	m_veh_Mx = -m_whl_Fy * m_arm + m_whl_Mx;
	m_veh_My = m_whl_Fx * m_arm + m_whl_My;
	m_veh_Mz = 0.0 + m_whl_Mz;
}

void Sus_Macpherson::output (d_vec &outputs){
	outputs[0] = m_veh_Fx;
	outputs[1] = m_veh_Fy;
	outputs[2] = m_veh_Fz;
	outputs[3] = m_veh_Mx;
	outputs[4] = m_veh_My;
	outputs[5] = m_veh_Mz;
	outputs[6] = m_whl_Fx;
	outputs[7] = m_whl_Fy;
	outputs[8] = m_whl_Fz;
	outputs[9] = m_whl_Vx;
	outputs[10] = m_whl_Vy;
	outputs[11] = m_whl_Vz;
	outputs[12] = m_whl_camber;
	outputs[13] = m_whl_caster;
	outputs[14] = m_whl_strg;
}

double Sus_Macpherson::calculate_hard_stop_force(const double \
&x_morp_hmax, const double &x_dot){
     double tmp = saturation(abs(4.0 * x_morp_hmax / (0.05 * m_Hmax)), \
     0.0, 4.0);
     return tanh(tmp) * pow(tmp, 3.0) * m_Kz * x_morp_hmax + tanh(tmp) \
     * m_Cz * x_dot;
}
