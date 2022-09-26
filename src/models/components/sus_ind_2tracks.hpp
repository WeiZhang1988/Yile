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
#ifndef SUS_INDEPENDENT_TWO_TRACKS_HPP
#define SUS_INDEPENDENT_TWO_TRACKS_HPP
#include "common.hpp"

namespace NMSPC{
class Sus_Ind_2Tracks {
public:
	static const int m_con_states_num = 0;									//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;					//amount of derivatives;
	static const int m_dis_states_num = 0;									//amount of discrete states;

	Sus_Ind_2Tracks (double F0z=2886.0, double Kz=52451.006579283188, double Cz=5565.226438019838, \
	double Hmax=0.25, double roll_strg_H_slp=-0.2269, \
	double toe=0.0349, double toe_strg_slp=0.01, \
	double caster=0.0698, double caster_H_slp=-0.2269, double caster_strg_slp=0.01, \
	double camber=0.0698, double camber_H_slp=-0.2269, double camber_strg_slp=0.01, \
	double strg_hgt_slp=0.1432,\
	double as_R=0.2, double as_ntrl_ang=0.5236, double as_trsK=8e2, \
	bool has_anti_sway=true, bool is_strg=true) :
	m_F0z_l(F0z), m_Kz_l(Kz), m_Cz_l(Cz), m_Hmax_l(Hmax), \
	m_roll_strg_H_slp_l(roll_strg_H_slp), \
	m_toe_l(toe), m_toe_strg_slp_l(toe_strg_slp), \
	m_caster_l(caster), m_caster_H_slp_l(caster_H_slp), \
	m_caster_strg_slp_l(caster_strg_slp), \
	m_camber_l(camber), m_camber_H_slp_l(camber_H_slp), \
	m_camber_strg_slp_l(camber_strg_slp), \
	m_strg_hgt_slp_l(strg_hgt_slp), \
	m_F0z_r(F0z), m_Kz_r(Kz), m_Cz_r(Cz), m_Hmax_r(Hmax), \
	m_roll_strg_H_slp_r(roll_strg_H_slp), \
	m_toe_r(toe), m_toe_strg_slp_r(toe_strg_slp), \
	m_caster_r(caster), m_caster_H_slp_r(caster_H_slp), \
	m_caster_strg_slp_r(caster_strg_slp), \
	m_camber_r(camber), m_camber_H_slp_r(camber_H_slp), \
	m_camber_strg_slp_r(camber_strg_slp), \
	m_strg_hgt_slp_r(strg_hgt_slp), \
	m_as_R(as_R), m_as_ntrl_ang(as_ntrl_ang), m_as_trsK(as_trsK), \
	m_has_anti_sway(has_anti_sway), m_is_strg(is_strg)
	{	
		if (!is_strg) {
			m_strg_hgt_slp_l = 0.0;
			m_strg_hgt_slp_r = 0.0;
			}
	}

	Sus_Ind_2Tracks &operator= (const Sus_Ind_2Tracks &org);

	void push_con_states (d_vec &con_states) {};
	void pull_con_states (const d_vec &con_states) {};

	void pull_pv (const double &Veh_hgt_cg, const double &Veh_r, \
	const double &Strg_str_l, const double &Sus_TirPz_l, const double &Sus_Tirvz_l,	const double &Tir_Re_l, \
	const double &Veh_Pz_l, const double &Veh_vx_l, const double &Veh_vy_l, const double &Veh_vz_l, \
	const double &Strg_str_r, const double &Sus_TirPz_r, const double &Sus_Tirvz_r,	const double &Tir_Re_r, \
	const double &Veh_Pz_r, const double &Veh_vx_r, const double &Veh_vy_r, const double &Veh_vz_r);
	void push_pv (double &Sus_str_l, double &Sus_gamma_l, double &Sus_caster_l, double &Sus_r_l, double &Sus_vx_l, double &Sus_vy_l, double &Sus_vz_l, \
	double &Sus_str_r, double &Sus_gamma_r, double &Sus_caster_r, double &Sus_r_r, double &Sus_vx_r, double &Sus_vy_r, double &Sus_vz_r);
	void pull_fm_z ();
	void push_fm_z (double &Sus_VehFz_l, double &Sus_Fz_l, double &Sus_VehFz_r, double &Sus_Fz_r);
	void pull_fm_o (const double &Sus_TirFx_l, const double &Sus_TirFy_l, const double &Tir_Mx_l, const double &Tir_My_l, const double &Tir_Mz_l, \
	const double &Sus_TirFx_r, const double &Sus_TirFy_r, const double &Tir_Mx_r, const double &Tir_My_r, const double &Tir_Mz_r);
	void push_fm_o (double &Sus_VehFx_l, double &Sus_VehFy_l, double &Sus_VehMx_l, double &Sus_VehMy_l, double &Sus_VehMz_l, \
	double &Sus_VehFx_r, double &Sus_VehFy_r, double &Sus_VehMx_r, double &Sus_VehMy_r, double &Sus_VehMz_r);
	void pull_fm (const double &Sus_TirFx_l, const double &Sus_TirFy_l, const double &Tir_Mx_l, const double &Tir_My_l, const double &Tir_Mz_l, \
	const double &Sus_TirFx_r, const double &Sus_TirFy_r, const double &Tir_Mx_r, const double &Tir_My_r, const double &Tir_Mz_r);
	void push_fm (double &Sus_VehFx_l, double &Sus_VehFy_l, double &Sus_VehFz_l, double &Sus_VehMx_l, double &Sus_VehMy_l, double &Sus_VehMz_l, double &Sus_Fz_l, \
	double &Sus_VehFx_r, double &Sus_VehFy_r, double &Sus_VehFz_r, double &Sus_VehMx_r, double &Sus_VehMy_r, double &Sus_VehMz_r, double &Sus_Fz_r);

	void update_drv (d_vec &outputs) {};

private:
	//built-in parameters
	piece_wise_linear uhsbm = piece_wise_linear({0.0, 0.01, 0.02},{0.0, 1.0, 1.0});
	piece_wise_linear lhsbm = piece_wise_linear({-0.02, -0.01, 0.0},{1.0, 1.0, 0.0});
	piece_wise_linear ang_tan_lmt = piece_wise_linear({-1.0, 1.0},{-1.0, 1.0});
	//parameters	
	// preload , spring constant , damping , maximum height
	double m_F0z_l, m_Kz_l, m_Cz_l, m_Hmax_l, \
	m_F0z_r, m_Kz_r, m_Cz_r, m_Hmax_r; 		
	// rolling steering angle vs suspension height
	double m_roll_strg_H_slp_l, \
	m_roll_strg_H_slp_r; 
	double m_toe_l, m_toe_strg_slp_l, \
	m_toe_r, m_toe_strg_slp_r; 		//
	double m_caster_l, m_caster_H_slp_l, m_caster_strg_slp_l, \
	m_caster_r, m_caster_H_slp_r, m_caster_strg_slp_r; 
	double m_camber_l, m_camber_H_slp_l, m_camber_strg_slp_l, \
	m_camber_r, m_camber_H_slp_r, m_camber_strg_slp_r; 
	//suspension height vs steering angle
	double m_strg_hgt_slp_l, m_strg_hgt_slp_r;
	//anti sway
	double m_as_R, m_as_ntrl_ang, m_as_trsK;
	bool m_has_anti_sway;
	// is steering 			
	bool m_is_strg; 				
	
	//inputs
	double m_Veh_hgt_cg		= NaN;
	double m_Veh_r			= NaN;
	//--left
	double m_Strg_str_l		= NaN;
	double m_Sus_TirPz_l	= NaN;
	double m_Sus_Tirvz_l	= NaN;
	double m_Tir_Re_l		= NaN;
	double m_Veh_Pz_l		= NaN;
	double m_Veh_vx_l		= NaN;
	double m_Veh_vy_l		= NaN;
	double m_Veh_vz_l		= NaN;
	double m_Sus_TirFx_l	= NaN;
	double m_Sus_TirFy_l	= NaN;
	double m_Tir_Mx_l		= NaN;
	double m_Tir_My_l		= NaN;
	double m_Tir_Mz_l		= NaN;
	//--right
	double m_Strg_str_r		= NaN;
	double m_Sus_TirPz_r	= NaN;
	double m_Sus_Tirvz_r	= NaN;
	double m_Tir_Re_r		= NaN;
	double m_Veh_Pz_r		= NaN;
	double m_Veh_vx_r		= NaN;
	double m_Veh_vy_r		= NaN;
	double m_Veh_vz_r		= NaN;
	double m_Sus_TirFx_r	= NaN;
	double m_Sus_TirFy_r	= NaN;
	double m_Tir_Mx_r		= NaN;
	double m_Tir_My_r		= NaN;
	double m_Tir_Mz_r		= NaN;

	//outputs
	//--left
	double m_Sus_str_l   = NaN;
	double m_Sus_gamma_l = NaN;
	double m_Sus_caster_l = NaN;
	double m_Sus_r_l = NaN;
	double m_Sus_vx_l = NaN;
	double m_Sus_vy_l = NaN;
	double m_Sus_vz_l = NaN;	//note: vx vy are vehicle hardpoint velocity, vz is tire vz in suspension frame
	double m_Sus_VehFx_l = NaN;
	double m_Sus_VehFy_l = NaN;	
	double m_Sus_VehFz_l = NaN;
	double m_Sus_VehMx_l = NaN;
	double m_Sus_VehMy_l = NaN;	
	double m_Sus_VehMz_l = NaN;	
	double m_Sus_Fz_l = NaN;
	//--right
	double m_Sus_str_r   = NaN;
	double m_Sus_gamma_r = NaN;
	double m_Sus_caster_r = NaN;
	double m_Sus_r_r = NaN;
	double m_Sus_vx_r = NaN;
	double m_Sus_vy_r = NaN;
	double m_Sus_vz_r = NaN;
	double m_Sus_VehFx_r = NaN;
	double m_Sus_VehFy_r = NaN;	
	double m_Sus_VehFz_r = NaN;
	double m_Sus_VehMx_r = NaN;
	double m_Sus_VehMy_r = NaN;	
	double m_Sus_VehMz_r = NaN;	
	double m_Sus_Fz_r = NaN;

	//middle variables
	//--left
	double m_Sus_VehPz_l = NaN;
	double m_Sus_Pz_l	= NaN;
	double m_Veh_hgt_l  = NaN;	//ambiguous name. actually it means the suppression amount without Fz0
	double m_Sus_hgt_l = NaN;	//ambiguous name. actually it means the total suppresion amount of suspension.
	double m_x_dot_l = NaN;
	double m_x_minus_hmax_l = NaN;
	double m_x_plus_hmax_l = NaN;
	double m_hard_stop_force_l = NaN;
	double m_total_effort_l = NaN;
	double m_adjusted_toe_l = NaN;
	double m_arm_l = NaN;
	//--right
	double m_Sus_VehPz_r = NaN;
	double m_Sus_Pz_r	= NaN;
	double m_Veh_hgt_r  = NaN;	//ambiguous name. actually it means the suppression amount without Fz0
	double m_Sus_hgt_r = NaN;	//ambiguous name. actually it means the total suppresion amount of suspension.
	double m_x_dot_r = NaN;
	double m_x_minus_hmax_r = NaN;
	double m_x_plus_hmax_r = NaN;
	double m_hard_stop_force_r = NaN;
	double m_total_effort_r = NaN;
	double m_adjusted_toe_r = NaN;
	double m_arm_r = NaN;

	//
	double calculate_hard_stop_force_max_stop(const double &x_minus_hmax, \
	const double &x_dot, const double &Hmax, const double &Kz, const double &Cz);	
	double calculate_hard_stop_force_min_stop(const double &x_plus_hmax, \
	const double &x_dot, const double &Hmax, const double &Kz, const double &Cz);
	void calculate_anti_sway_force_in_position(double &whlFz_l, double &whlFz_r, double &vehFz_l, double &vehFz_r, \
	const double &whlZ_l, const double &whlZ_r, const double &vehZ_l, const double &vehZ_r);
};

}	//end of name space
#endif //SUS_INDEPENDENT_TWO_TRACKS_HPP
