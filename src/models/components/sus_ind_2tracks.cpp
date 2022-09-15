// =============================================================================
// PROJECT YILE - 
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of this source code is governed by a GPL-3.0 license that can be found
// in the LICENSE file
//
// Author of this file	Wei ZHANG wei_zhang_1988@outlook.com
//
// =============================================================================
#include "sus_ind_2tracks.hpp"

void NMSPC::Sus_Ind_2Tracks::update_pv (const d_vec &inputs, d_vec &outputs) {
	//pull inputs
	//--left
	m_Strg_ang_l = inputs[0];
	m_Whl_Pz_l   = inputs[1];
	m_Whl_Vz_l   = inputs[2];
	m_Whl_Re_l   = inputs[3];
	m_Veh_Pz_l   = inputs[4];
	m_Veh_Vx_l   = inputs[5];
	m_Veh_Vy_l   = inputs[6];
	m_Veh_Vz_l   = inputs[7];
	//--right
	m_Strg_ang_r = inputs[8];
	m_Whl_Pz_r   = inputs[9];
	m_Whl_Vz_r   = inputs[10];
	m_Whl_Re_r   = inputs[11];
	m_Veh_Pz_r   = inputs[12];
	m_Veh_Vx_r   = inputs[13];
	m_Veh_Vy_r   = inputs[14];
	m_Veh_Vz_r   = inputs[15];
	
	//process	
	//--left
	m_Sus_Vx_l = m_Veh_Vx_l;
	m_Sus_Vy_l = m_Veh_Vy_l;
	m_Sus_Vz_l = -m_Whl_Vz_l;
	m_Veh_h_l = -(abs(m_Strg_ang_l) * m_strg_hgt_slp_l - m_Whl_Pz_l + m_Veh_Pz_l);
	m_sus_h_l = -(m_F0z_l / m_Kz_l - m_Veh_h_l);
	m_x_minus_hmax_l = -m_sus_h_l - m_Hmax_l;
	m_x_plus_hmax_l  = -m_sus_h_l + m_Hmax_l;
	m_x_dot_l 	     = m_Veh_Vz_l - m_Whl_Vz_l;
	m_adjusted_toe_l = -(abs(m_Strg_ang_l) * m_toe_strg_slp_l + m_toe_l + 
	m_roll_strg_H_slp_l * m_sus_h_l);
	m_Whl_strg_l = m_adjusted_toe_l - m_toe_l + m_Strg_ang_l;
	m_Whl_camber_l = abs(m_Strg_ang_l) * m_camber_strg_slp_l + m_camber_l + \
	m_camber_H_slp_l * m_sus_h_l;
	m_Whl_caster_l = abs(m_Strg_ang_l) * m_caster_strg_slp_l + m_caster_l + \
	m_caster_H_slp_l * m_sus_h_l;
	m_arm_l = m_Veh_h_l + m_Whl_Re_l;
	//--right
	m_Sus_Vx_r = m_Veh_Vx_r;
	m_Sus_Vy_r = m_Veh_Vy_r;
	m_Sus_Vz_r = -m_Whl_Vz_r;
	m_Veh_h_r = -(abs(m_Strg_ang_r) * m_strg_hgt_slp_r - m_Whl_Pz_r + m_Veh_Pz_r);
	m_sus_h_r = -(m_F0z_r / m_Kz_r - m_Veh_h_r);
	m_x_minus_hmax_r = -m_sus_h_r - m_Hmax_r;
	m_x_plus_hmax_r  = -m_sus_h_r + m_Hmax_r;
	m_x_dot_r 	     = m_Veh_Vz_r - m_Whl_Vz_r;
	m_adjusted_toe_r = abs(m_Strg_ang_r) * m_toe_strg_slp_r + m_toe_r + \
	m_roll_strg_H_slp_r * m_sus_h_r;
	m_Whl_strg_r = m_adjusted_toe_r - m_toe_r + m_Strg_ang_r;
	m_Whl_camber_r = abs(m_Strg_ang_r) * m_camber_strg_slp_r + m_camber_r + \
	m_camber_H_slp_r * m_sus_h_r;
	m_Whl_caster_r = abs(m_Strg_ang_r) * m_caster_strg_slp_r + m_caster_r + \
	m_caster_H_slp_r * m_sus_h_r;
	m_arm_r = m_Veh_h_r + m_Whl_Re_r;
	
	//push outputs
	outputs[0] = m_Whl_strg_l;
	outputs[1] = m_Whl_camber_l;
	outputs[2] = m_Whl_caster_l;
	outputs[3] = m_Veh_Vx_l;
	outputs[4] = m_Veh_Vy_l;
	outputs[5] = m_Whl_strg_r;
	outputs[6] = m_Whl_camber_r;
	outputs[7] = m_Whl_caster_r;
	outputs[8] = m_Veh_Vx_r;
	outputs[9] = m_Veh_Vy_r;
}

void NMSPC::Sus_Ind_2Tracks::update_fm (const d_vec &inputs, d_vec &outputs) {
	//pull inputs
	//--left
	m_Whl_Fx_l   = inputs[0];
	m_Whl_Fy_l   = inputs[1];
	m_Whl_Mx_l   = inputs[2];
	m_Whl_My_l   = inputs[3];
	m_Whl_Mz_l   = inputs[4];
	//--right
	m_Whl_Fx_r   = inputs[5];
	m_Whl_Fy_r   = inputs[6];
	m_Whl_Mx_r   = inputs[7];
	m_Whl_My_r   = inputs[8];
	m_Whl_Mz_r   = inputs[9];
	//process
	//--left
	m_Veh_Fx_l = m_Whl_Fx_l;
	m_Veh_Fy_l = m_Whl_Fy_l;
	if ((m_x_minus_hmax_l) > 0.0) {
		// upper hard stop
		m_hard_stop_force_l = calculate_hard_stop_force(m_x_minus_hmax_l, \
		 m_x_dot_l, m_Hmax_l, m_Kz_l, m_Cz_l);
	} else if ((m_x_plus_hmax_l) < 0.0) {
		// lower hard stop
		m_hard_stop_force_l = calculate_hard_stop_force(m_x_plus_hmax_l, \
		 m_x_dot_l, m_Hmax_l, m_Kz_l, m_Cz_l);
	} else {
		m_hard_stop_force_l = 0.0;
	}
	m_total_effort_l = (-m_sus_h_l * m_Kz_l) + ((m_Veh_Vz_l - m_Whl_Vz_l)*m_Cz_l) + \
	m_hard_stop_force_l;
	m_Veh_Fz_l = -m_total_effort_l;
	m_Whl_Fz_l = m_total_effort_l;
	m_Veh_Mx_l = -m_Whl_Fy_l * m_arm_l + m_Whl_Mx_l;
	m_Veh_My_l = m_Whl_Fx_l * m_arm_l + m_Whl_My_l;
	m_Veh_Mz_l = 0.0 + m_Whl_Mz_l;
	//--right
	m_Veh_Fx_r = m_Whl_Fx_r;
	m_Veh_Fy_r = m_Whl_Fy_r;
	if ((m_x_minus_hmax_r) > 0.0) {
		// upper hard stop
		m_hard_stop_force_r = calculate_hard_stop_force(m_x_minus_hmax_r, \
		 m_x_dot_r, m_Hmax_r, m_Kz_r, m_Cz_r);
	} else if ((m_x_plus_hmax_r) < 0.0) {
		// lower hard stop
		m_hard_stop_force_r = calculate_hard_stop_force(m_x_plus_hmax_r, \
		 m_x_dot_r, m_Hmax_r, m_Kz_r, m_Cz_r);
	} else {
		m_hard_stop_force_r = 0.0;
	}
	m_total_effort_r = (-m_sus_h_r * m_Kz_r) + ((m_Veh_Vz_r - m_Whl_Vz_r)*m_Cz_r) + \
	m_hard_stop_force_r;
	m_Veh_Fz_r = -m_total_effort_r;
	m_Whl_Fz_r = m_total_effort_r;
	m_Veh_Mx_r = -m_Whl_Fy_r * m_arm_r + m_Whl_Mx_r;
	m_Veh_My_r = m_Whl_Fx_r * m_arm_r + m_Whl_My_r;
	m_Veh_Mz_r = 0.0 + m_Whl_Mz_r;

	//push outputs
	outputs[0] = m_Veh_Fx_l;
	outputs[1] = m_Veh_Fy_l;	
	outputs[2] = m_Veh_Fz_l;
	outputs[3] = m_Veh_Mx_l;
	outputs[4] = m_Veh_My_l;
	outputs[5] = m_Veh_Mz_l;
	outputs[6] = m_Whl_Fz_l;
	outputs[7] = m_Veh_Fx_r;
	outputs[8] = m_Veh_Fy_r;	
	outputs[9] = m_Veh_Fz_r;
	outputs[10] = m_Veh_Mx_r;
	outputs[11] = m_Veh_My_r;
	outputs[12] = m_Veh_Mz_r;
	outputs[13] = m_Whl_Fz_r;
}

double NMSPC::Sus_Ind_2Tracks::calculate_hard_stop_force(const double \
&x_morp_hmax, const double &x_dot, const double &Hmax, const double &Kz, const double &Cz){
     double tmp = saturation(abs(4.0 * x_morp_hmax / (0.05 * Hmax)), \
     0.0, 4.0);
     return tanh(tmp) * pow(tmp, 3.0) * Kz * x_morp_hmax + tanh(tmp) \
     * Cz * x_dot;
}
