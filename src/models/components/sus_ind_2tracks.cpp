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

void NMSPC::Sus_Ind_2Tracks::pull_pv (const double &Veh_hgt_cg, const double &Veh_r, const double &Strg_str_l, const double &Sus_TirPz_l, const double &Sus_Tirvz_l,	const double &Tir_Re_l, \
	const double &Veh_Pz_l, const double &Veh_vx_l, const double &Veh_vy_l, const double &Veh_vz_l, \
	const double &Strg_str_r, const double &Sus_TirPz_r, const double &Sus_Tirvz_r,	const double &Tir_Re_r, \
	const double &Veh_Pz_r, const double &Veh_vx_r, const double &Veh_vy_r, const double &Veh_vz_r) {
	//pull inputs
	m_Veh_hgt_cg = Veh_hgt_cg;
	m_Veh_r 	 = Veh_r;
	//--left
	m_Strg_str_l = Strg_str_l;
	m_Sus_TirPz_l= Sus_TirPz_l;
	m_Sus_Tirvz_l= Sus_Tirvz_l;
	m_Tir_Re_l   = Tir_Re_l;
	m_Veh_Pz_l   = Veh_Pz_l;
	m_Veh_vx_l   = Veh_vx_l;
	m_Veh_vy_l   = Veh_vy_l;
	m_Veh_vz_l   = Veh_vz_l;
	//--right
	m_Strg_str_r = Strg_str_r;
	m_Sus_TirPz_r= Sus_TirPz_r;
	m_Sus_Tirvz_r= Sus_Tirvz_r;
	m_Tir_Re_r   = Tir_Re_r;
	m_Veh_Pz_r   = Veh_Pz_r;
	m_Veh_vx_r   = Veh_vx_r;
	m_Veh_vy_r   = Veh_vy_r;
	m_Veh_vz_r   = Veh_vz_r;
	
	//process	
	//--left
	m_Sus_r_l  = m_Veh_r;			// direct output
	m_Sus_vx_l = m_Veh_vx_l;		// direct output
	m_Sus_vy_l = m_Veh_vy_l;		// direct output
	m_Sus_vz_l = m_Sus_Tirvz_l;		// direct output
	m_Sus_VehPz_l = m_Veh_Pz_l - m_Veh_hgt_cg;
	m_Veh_hgt_l = -(abs(m_Strg_str_l) * m_strg_hgt_slp_l - m_Sus_TirPz_l + m_Sus_VehPz_l);
	m_Sus_hgt_l = -(m_F0z_l / m_Kz_l - m_Veh_hgt_l);
	m_x_minus_hmax_l = -m_Sus_hgt_l - m_Hmax_l;
	m_x_plus_hmax_l  = -m_Sus_hgt_l + m_Hmax_l;
	m_x_dot_l 	     = m_Veh_vz_l - m_Sus_Tirvz_l;
	m_adjusted_toe_l = (abs(m_Strg_str_l) * m_toe_strg_slp_l + m_toe_l + 
	m_roll_strg_H_slp_l * m_Sus_hgt_l);//左边是正的，右边是负的
	m_Sus_str_l = m_adjusted_toe_l - m_toe_l + m_Strg_str_l;//++++
	m_Sus_gamma_l = abs(m_Strg_str_l) * m_camber_strg_slp_l + m_camber_l + \
	m_camber_H_slp_l * m_Sus_hgt_l;
	m_Sus_caster_l = abs(m_Strg_str_l) * m_caster_strg_slp_l + m_caster_l + \
	m_caster_H_slp_l * m_Sus_hgt_l;
	m_arm_l = m_Veh_hgt_l + m_Tir_Re_l;
	//--right
	m_Sus_r_r  = m_Veh_r;
	m_Sus_vx_r = m_Veh_vx_r;
	m_Sus_vy_r = m_Veh_vy_r;
	m_Sus_vz_r = m_Sus_Tirvz_r;
	m_Sus_VehPz_r = m_Veh_Pz_r - m_Veh_hgt_cg;
	m_Veh_hgt_r = -(abs(m_Strg_str_r) * m_strg_hgt_slp_r - m_Sus_TirPz_r + m_Sus_VehPz_r);
	m_Sus_hgt_r = -(m_F0z_r / m_Kz_r - m_Veh_hgt_r);
	m_x_minus_hmax_r = -m_Sus_hgt_r - m_Hmax_r;
	m_x_plus_hmax_r  = -m_Sus_hgt_r + m_Hmax_r;
	m_x_dot_r 	     = m_Veh_vz_r - m_Sus_Tirvz_r;
	m_adjusted_toe_r = -(abs(m_Strg_str_r) * m_toe_strg_slp_r + m_toe_r + \
	m_roll_strg_H_slp_r * m_Sus_hgt_r);//右边是负的
	m_Sus_str_r = m_adjusted_toe_r - m_toe_r + m_Strg_str_r;
	m_Sus_gamma_r = abs(m_Strg_str_r) * m_camber_strg_slp_r + m_camber_r + \
	m_camber_H_slp_r * m_Sus_hgt_r;
	m_Sus_caster_r = abs(m_Strg_str_r) * m_caster_strg_slp_r + m_caster_r + \
	m_caster_H_slp_r * m_Sus_hgt_r;
	m_arm_r = m_Veh_hgt_r + m_Tir_Re_r;
	
}

void NMSPC::Sus_Ind_2Tracks::push_pv(double &Sus_str_l, double &Sus_gamma_l, double &Sus_caster_l, double &Sus_r_l, double &Sus_vx_l, double &Sus_vy_l, double &Sus_vz_l, \
double &Sus_str_r, double &Sus_gamma_r, double &Sus_caster_r, double &Sus_r_r, double &Sus_vx_r, double &Sus_vy_r, double &Sus_vz_r) {
	Sus_str_l = m_Sus_str_l;
	Sus_gamma_l = m_Sus_gamma_l;
	Sus_caster_l = m_Sus_caster_l;
	Sus_r_l = m_Sus_r_l;
	Sus_vx_l = m_Sus_vx_l;
	Sus_vy_l = m_Sus_vy_l;
	Sus_vz_l = m_Sus_vz_l;
	Sus_str_r = m_Sus_str_r;
	Sus_gamma_r = m_Sus_gamma_r;
	Sus_caster_r = m_Sus_caster_r;
	Sus_r_r = m_Sus_r_r;
	Sus_vx_r = m_Sus_vx_r;
	Sus_vy_r = m_Sus_vy_r;
	Sus_vz_r = m_Sus_vz_r;
}

void NMSPC::Sus_Ind_2Tracks::pull_fm_z() {
	//process
	//--left
	if ((m_x_minus_hmax_l) > 0.0) {
		// upper hard stop
		m_hard_stop_force_l = calculate_hard_stop_force_max_stop(m_x_minus_hmax_l, \
		 m_x_dot_l, m_Hmax_l, m_Kz_l, m_Cz_l);
	} else if ((m_x_plus_hmax_l) < 0.0) {
		// lower hard stop
		m_hard_stop_force_l = calculate_hard_stop_force_min_stop(m_x_plus_hmax_l, \
		 m_x_dot_l, m_Hmax_l, m_Kz_l, m_Cz_l);
	} else {
		m_hard_stop_force_l = 0.0;
	}
	m_total_effort_l = (-m_Sus_hgt_l * m_Kz_l) + (m_x_dot_l* m_Cz_l) + m_hard_stop_force_l;
	m_Sus_VehFz_l = -m_total_effort_l;
	m_Sus_Fz_l = m_total_effort_l;

	//--right
	if ((m_x_minus_hmax_r) > 0.0) {
		// upper hard stop
		m_hard_stop_force_r = calculate_hard_stop_force_max_stop(m_x_minus_hmax_r, \
		 m_x_dot_r, m_Hmax_r, m_Kz_r, m_Cz_r);
	} else if ((m_x_plus_hmax_r) < 0.0) {
		// lower hard stop
		m_hard_stop_force_r = calculate_hard_stop_force_min_stop(m_x_plus_hmax_r, \
		 m_x_dot_r, m_Hmax_r, m_Kz_r, m_Cz_r);
	} else {
		m_hard_stop_force_r = 0.0;
	}
	m_total_effort_r = (-m_Sus_hgt_r * m_Kz_r) + (m_x_dot_r*m_Cz_r) + m_hard_stop_force_r;
	m_Sus_VehFz_r = -m_total_effort_r;
	m_Sus_Fz_r = m_total_effort_r;

	if (m_has_anti_sway)
		calculate_anti_sway_force_in_position(m_Sus_Fz_l, m_Sus_Fz_r, m_Sus_VehFz_l, m_Sus_VehFz_r, \
		m_Sus_TirPz_l, m_Sus_TirPz_r, m_Veh_Pz_l, m_Veh_Pz_r);

}

void NMSPC::Sus_Ind_2Tracks::pull_fm_o(const double &Sus_TirFx_l, const double &Sus_TirFy_l, const double &Tir_Mx_l, const double &Tir_My_l, const double &Tir_Mz_l, \
const double &Sus_TirFx_r, const double &Sus_TirFy_r, const double &Tir_Mx_r, const double &Tir_My_r, const double &Tir_Mz_r) {
	//pull inputs
	//--left
	m_Sus_TirFx_l   = Sus_TirFx_l;
	m_Sus_TirFy_l   = Sus_TirFy_l;
	m_Tir_Mx_l      = Tir_Mx_l;
	m_Tir_My_l      = Tir_My_l;
	m_Tir_Mz_l      = Tir_Mz_l;
	//--right
	m_Sus_TirFx_r   = Sus_TirFx_r;
	m_Sus_TirFy_r   = Sus_TirFy_r;
	m_Tir_Mx_r      = Tir_Mx_r;
	m_Tir_My_r      = Tir_My_r;
	m_Tir_Mz_r      = Tir_Mz_r;
	//process
	//--left
	m_Sus_VehFx_l = m_Sus_TirFx_l;
	m_Sus_VehFy_l = m_Sus_TirFy_l;
	m_Sus_VehMx_l = -m_Sus_TirFy_l * m_arm_l + m_Tir_Mx_l;
	m_Sus_VehMy_l = m_Sus_TirFx_l * m_arm_l + m_Tir_My_l;
	m_Sus_VehMz_l = 0.0 + m_Tir_Mz_l;
	//--right
	m_Sus_VehFx_r = m_Sus_TirFx_r;
	m_Sus_VehFy_r = m_Sus_TirFy_r;
	m_Sus_VehMx_r = -m_Sus_TirFy_r * m_arm_r + m_Tir_Mx_r;
	m_Sus_VehMy_r = m_Sus_TirFx_r * m_arm_r + m_Tir_My_r;
	m_Sus_VehMz_r = 0.0 + m_Tir_Mz_r;

}

void NMSPC::Sus_Ind_2Tracks::pull_fm(const double &Sus_TirFx_l, const double &Sus_TirFy_l, const double &Tir_Mx_l, const double &Tir_My_l, const double &Tir_Mz_l, \
const double &Sus_TirFx_r, const double &Sus_TirFy_r, const double &Tir_Mx_r, const double &Tir_My_r, const double &Tir_Mz_r) {
	pull_fm_z();
	pull_fm_o(Sus_TirFx_l, Sus_TirFy_l, Tir_Mx_l, Tir_My_l, Tir_Mz_l, Sus_TirFx_r, Sus_TirFy_r, Tir_Mx_r, Tir_My_r, Tir_Mz_r);
}

void NMSPC::Sus_Ind_2Tracks::push_fm_z(double &Sus_VehFz_l, double &Sus_Fz_l, double &Sus_VehFz_r, double &Sus_Fz_r) {
	Sus_VehFz_l = m_Sus_VehFz_l;
	Sus_Fz_l 	= m_Sus_Fz_l;
	Sus_VehFz_r = m_Sus_VehFz_r;
	Sus_Fz_r 	= m_Sus_Fz_r;
}

void NMSPC::Sus_Ind_2Tracks::push_fm_o(double &Sus_VehFx_l, double &Sus_VehFy_l, double &Sus_VehMx_l, double &Sus_VehMy_l, double &Sus_VehMz_l, \
double &Sus_VehFx_r, double &Sus_VehFy_r, double &Sus_VehMx_r, double &Sus_VehMy_r, double &Sus_VehMz_r) {
	Sus_VehFx_l = m_Sus_VehFx_l;
	Sus_VehFy_l = m_Sus_VehFy_l;
	Sus_VehMx_l = m_Sus_VehMx_l;
	Sus_VehMy_l = m_Sus_VehMy_l;
	Sus_VehMz_l = m_Sus_VehMz_l;
	Sus_VehFx_r = m_Sus_VehFx_r;
	Sus_VehFy_r = m_Sus_VehFy_r;
	Sus_VehMx_r = m_Sus_VehMx_r;
	Sus_VehMy_r = m_Sus_VehMy_r;
	Sus_VehMz_r = m_Sus_VehMz_r;
	
}

void NMSPC::Sus_Ind_2Tracks::push_fm(double &Sus_VehFx_l, double &Sus_VehFy_l, double &Sus_VehFz_l, double &Sus_VehMx_l, double &Sus_VehMy_l, double &Sus_VehMz_l, double &Sus_Fz_l, \
double &Sus_VehFx_r, double &Sus_VehFy_r, double &Sus_VehFz_r, double &Sus_VehMx_r, double &Sus_VehMy_r, double &Sus_VehMz_r, double &Sus_Fz_r) {
	push_fm_z(Sus_VehFz_l, Sus_Fz_l, Sus_VehFz_r, Sus_Fz_r);
	push_fm_o(Sus_VehFx_l, Sus_VehFy_l, Sus_VehMx_l, Sus_VehMy_l, Sus_VehMz_l, \
	Sus_VehFx_r, Sus_VehFy_r, Sus_VehMx_r, Sus_VehMy_r, Sus_VehMz_r);
}


double NMSPC::Sus_Ind_2Tracks::calculate_hard_stop_force_max_stop(const double \
&x_minus_hmax, const double &x_dot, const double &Hmax, const double &Kz, const double &Cz){
     double tmp = saturation(abs(4.0 * x_minus_hmax / (0.05 * Hmax)), \
     0.0, 4.0);
	 return (tanh(tmp) * pow(tmp, 3.0) * Kz * (x_minus_hmax) + tanh(tmp) \
     * Cz * x_dot * 3.0) * uhsbm(x_minus_hmax);
}

double NMSPC::Sus_Ind_2Tracks::calculate_hard_stop_force_min_stop(const double \
&x_plus_hmax, const double &x_dot, const double &Hmax, const double &Kz, const double &Cz){
     double tmp = saturation(abs(4.0 * x_plus_hmax / (0.05 * Hmax)), \
     0.0, 4.0);
	  return (tanh(tmp) * pow(tmp, 3.0) * Kz * (x_plus_hmax) + tanh(tmp) \
     * Cz * x_dot * 3.0) * lhsbm(x_plus_hmax);
}


void NMSPC::Sus_Ind_2Tracks::calculate_anti_sway_force_in_position(double &whlFz_l, double &whlFz_r, double &vehFz_l, double &vehFz_r, \
	const double &whlZ_l, const double &whlZ_r, const double &vehZ_l, const double &vehZ_r) {
	double ang_l = m_as_ntrl_ang - atan(ang_tan_lmt((m_as_R * tan(m_as_ntrl_ang) - (whlZ_l - vehZ_l)) / m_as_R));
	double ang_r = m_as_ntrl_ang - atan(ang_tan_lmt((m_as_R * tan(m_as_ntrl_ang) - (whlZ_r - vehZ_r)) / m_as_R));
	double tmp = (ang_l - ang_r) * m_as_trsK / m_as_R;
	whlFz_l -= tmp * cos(ang_l);
	whlFz_r -= tmp * cos(ang_r);
	vehFz_l += tmp * cos(ang_l);
	vehFz_r += tmp * cos(ang_r);
}