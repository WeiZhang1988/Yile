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

	Sus_Ind_2Tracks (real_Y F0z=2886.0, real_Y Kz=52451.006579283188, real_Y Cz=5565.224638019838, \
	real_Y Hmax=0.25, real_Y roll_strg_H_slp=-0.2269, \
	real_Y toe=0.0349, real_Y toe_strg_slp=0.01, \
	real_Y caster=0.0698, real_Y caster_H_slp=-0.2269, real_Y caster_strg_slp=0.01, \
	real_Y camber=0.0698, real_Y camber_H_slp=-0.2269, real_Y camber_strg_slp=0.01, \
	real_Y strg_hgt_slp=0.1432,\
	real_Y as_R=0.2, real_Y as_ntrl_ang=0.5236, real_Y as_trsK=8e2, \
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

	void pull_pv (const real_Y &Veh_hgt_cg, const real_Y &Veh_r, \
	const real_Y &Strg_str_l, const real_Y &Sus_TirPz_l, const real_Y &Sus_Tirvz_l,	const real_Y &Tir_Re_l, \
	const real_Y &Int_Pz_l, const real_Y &Int_Vz_l, \
	const real_Y &Veh_vx_l, const real_Y &Veh_vy_l, const real_Y &Veh_vz_l, \
	const real_Y &Strg_str_r, const real_Y &Sus_TirPz_r, const real_Y &Sus_Tirvz_r,	const real_Y &Tir_Re_r, \
	const real_Y &Int_Pz_r, const real_Y &Int_Vz_r, \
	const real_Y &Veh_vx_r, const real_Y &Veh_vy_r, const real_Y &Veh_vz_r);
	void push_pv (real_Y &Sus_str_l, real_Y &Sus_gamma_l, real_Y &Sus_caster_l, real_Y &Sus_r_l, real_Y &Sus_vx_l, real_Y &Sus_vy_l, real_Y &Sus_vz_l, \
	real_Y &Sus_str_r, real_Y &Sus_gamma_r, real_Y &Sus_caster_r, real_Y &Sus_r_r, real_Y &Sus_vx_r, real_Y &Sus_vy_r, real_Y &Sus_vz_r);
	void pull_fm_z ();
	void push_fm_z (real_Y &Sus_VehFz_l, real_Y &Sus_Fz_l, real_Y &Sus_VehFz_r, real_Y &Sus_Fz_r);
	void pull_fm_o (const real_Y &Sus_TirFx_l, const real_Y &Sus_TirFy_l, const real_Y &Tir_Mx_l, const real_Y &Tir_My_l, const real_Y &Tir_Mz_l, \
	const real_Y &Sus_TirFx_r, const real_Y &Sus_TirFy_r, const real_Y &Tir_Mx_r, const real_Y &Tir_My_r, const real_Y &Tir_Mz_r);
	void push_fm_o (real_Y &Sus_VehFx_l, real_Y &Sus_VehFy_l, real_Y &Sus_VehMx_l, real_Y &Sus_VehMy_l, real_Y &Sus_VehMz_l, \
	real_Y &Sus_VehFx_r, real_Y &Sus_VehFy_r, real_Y &Sus_VehMx_r, real_Y &Sus_VehMy_r, real_Y &Sus_VehMz_r);
	void pull_fm (const real_Y &Sus_TirFx_l, const real_Y &Sus_TirFy_l, const real_Y &Tir_Mx_l, const real_Y &Tir_My_l, const real_Y &Tir_Mz_l, \
	const real_Y &Sus_TirFx_r, const real_Y &Sus_TirFy_r, const real_Y &Tir_Mx_r, const real_Y &Tir_My_r, const real_Y &Tir_Mz_r);
	void push_fm (real_Y &Sus_VehFx_l, real_Y &Sus_VehFy_l, real_Y &Sus_VehFz_l, real_Y &Sus_VehMx_l, real_Y &Sus_VehMy_l, real_Y &Sus_VehMz_l, real_Y &Sus_Fz_l, \
	real_Y &Sus_VehFx_r, real_Y &Sus_VehFy_r, real_Y &Sus_VehFz_r, real_Y &Sus_VehMx_r, real_Y &Sus_VehMy_r, real_Y &Sus_VehMz_r, real_Y &Sus_Fz_r);

	void update_drv (d_vec &outputs) {};

private:
	//built-in parameters
	piece_wise_linear uhsbm = piece_wise_linear({0.0, 0.01, 0.02},{0.0, 1.0, 1.0});
	piece_wise_linear lhsbm = piece_wise_linear({-0.02, -0.01, 0.0},{1.0, 1.0, 0.0});
	piece_wise_linear ang_tan_lmt = piece_wise_linear({-1.0, 1.0},{-1.0, 1.0});
	//parameters	
	// preload , spring constant , damping , maximum height
	real_Y m_F0z_l, m_Kz_l, m_Cz_l, m_Hmax_l, \
	m_F0z_r, m_Kz_r, m_Cz_r, m_Hmax_r; 		
	// rolling steering angle vs suspension height
	real_Y m_roll_strg_H_slp_l, \
	m_roll_strg_H_slp_r; 
	real_Y m_toe_l, m_toe_strg_slp_l, \
	m_toe_r, m_toe_strg_slp_r; 		//
	real_Y m_caster_l, m_caster_H_slp_l, m_caster_strg_slp_l, \
	m_caster_r, m_caster_H_slp_r, m_caster_strg_slp_r; 
	real_Y m_camber_l, m_camber_H_slp_l, m_camber_strg_slp_l, \
	m_camber_r, m_camber_H_slp_r, m_camber_strg_slp_r; 
	//suspension height vs steering angle
	real_Y m_strg_hgt_slp_l, m_strg_hgt_slp_r;
	//anti sway
	real_Y m_as_R, m_as_ntrl_ang, m_as_trsK;
	bool m_has_anti_sway;
	// is steering 			
	bool m_is_strg; 				
	
	//inputs
	real_Y m_Veh_hgt_cg		= NaN;
	real_Y m_Veh_r			= NaN;
	//--left
	real_Y m_Strg_str_l		= NaN;
	real_Y m_Sus_TirPz_l	= NaN;
	real_Y m_Sus_Tirvz_l	= NaN;
	real_Y m_Tir_Re_l		= NaN;
	real_Y m_Int_Pz_l		= NaN;
	real_Y m_Int_Vz_l		= NaN;
	real_Y m_Veh_vx_l		= NaN;
	real_Y m_Veh_vy_l		= NaN;
	real_Y m_Veh_vz_l		= NaN;
	real_Y m_Sus_TirFx_l	= NaN;
	real_Y m_Sus_TirFy_l	= NaN;
	real_Y m_Tir_Mx_l		= NaN;
	real_Y m_Tir_My_l		= NaN;
	real_Y m_Tir_Mz_l		= NaN;
	//--right
	real_Y m_Strg_str_r		= NaN;
	real_Y m_Sus_TirPz_r	= NaN;
	real_Y m_Sus_Tirvz_r	= NaN;
	real_Y m_Tir_Re_r		= NaN;
	real_Y m_Int_Pz_r		= NaN;
	real_Y m_Int_Vz_r		= NaN;
	real_Y m_Veh_vx_r		= NaN;
	real_Y m_Veh_vy_r		= NaN;
	real_Y m_Veh_vz_r		= NaN;
	real_Y m_Sus_TirFx_r	= NaN;
	real_Y m_Sus_TirFy_r	= NaN;
	real_Y m_Tir_Mx_r		= NaN;
	real_Y m_Tir_My_r		= NaN;
	real_Y m_Tir_Mz_r		= NaN;

	//outputs
	//--left
	real_Y m_Sus_str_l   = NaN;
	real_Y m_Sus_gamma_l = NaN;
	real_Y m_Sus_caster_l = NaN;
	real_Y m_Sus_r_l = NaN;
	real_Y m_Sus_vx_l = NaN;
	real_Y m_Sus_vy_l = NaN;
	real_Y m_Sus_vz_l = NaN;	//note: vx vy are vehicle hardpoint velocity, vz is tire vz in suspension frame
	real_Y m_Sus_VehFx_l = NaN;
	real_Y m_Sus_VehFy_l = NaN;	
	real_Y m_Sus_VehFz_l = NaN;
	real_Y m_Sus_VehMx_l = NaN;
	real_Y m_Sus_VehMy_l = NaN;	
	real_Y m_Sus_VehMz_l = NaN;	
	real_Y m_Sus_Fz_l = NaN;
	//--right
	real_Y m_Sus_str_r   = NaN;
	real_Y m_Sus_gamma_r = NaN;
	real_Y m_Sus_caster_r = NaN;
	real_Y m_Sus_r_r = NaN;
	real_Y m_Sus_vx_r = NaN;
	real_Y m_Sus_vy_r = NaN;
	real_Y m_Sus_vz_r = NaN;
	real_Y m_Sus_VehFx_r = NaN;
	real_Y m_Sus_VehFy_r = NaN;	
	real_Y m_Sus_VehFz_r = NaN;
	real_Y m_Sus_VehMx_r = NaN;
	real_Y m_Sus_VehMy_r = NaN;	
	real_Y m_Sus_VehMz_r = NaN;	
	real_Y m_Sus_Fz_r = NaN;

	//middle variables
	//--left
	real_Y m_Sus_IntPz_l = NaN;
	real_Y m_Sus_IntVz_l = NaN;
	real_Y m_Sus_Pz_l	= NaN;
	real_Y m_Veh_hgt_l  = NaN;	//ambiguous name. actually it means the suppression amount without Fz0
	real_Y m_Sus_hgt_l = NaN;	//ambiguous name. actually it means the total suppresion amount of suspension.
	real_Y m_x_dot_l = NaN;
	real_Y m_x_minus_hmax_l = NaN;
	real_Y m_x_plus_hmax_l = NaN;
	real_Y m_hard_stop_force_l = NaN;
	real_Y m_total_effort_l = NaN;
	real_Y m_adjusted_toe_l = NaN;
	real_Y m_arm_l = NaN;
	//--right
	real_Y m_Sus_IntPz_r = NaN;
	real_Y m_Sus_IntVz_r = NaN;
	real_Y m_Sus_Pz_r	= NaN;
	real_Y m_Veh_hgt_r  = NaN;	//ambiguous name. actually it means the suppression amount without Fz0
	real_Y m_Sus_hgt_r = NaN;	//ambiguous name. actually it means the total suppresion amount of suspension.
	real_Y m_x_dot_r = NaN;
	real_Y m_x_minus_hmax_r = NaN;
	real_Y m_x_plus_hmax_r = NaN;
	real_Y m_hard_stop_force_r = NaN;
	real_Y m_total_effort_r = NaN;
	real_Y m_adjusted_toe_r = NaN;
	real_Y m_arm_r = NaN;

	//
	real_Y calculate_hard_stop_force_max_stop(const real_Y &x_minus_hmax, \
	const real_Y &x_dot, const real_Y &Hmax, const real_Y &Kz, const real_Y &Cz);	
	real_Y calculate_hard_stop_force_min_stop(const real_Y &x_plus_hmax, \
	const real_Y &x_dot, const real_Y &Hmax, const real_Y &Kz, const real_Y &Cz);
	void calculate_anti_sway_force_in_position(real_Y &whlFz_l, real_Y &whlFz_r, real_Y &vehFz_l, real_Y &vehFz_r, \
	const real_Y &whlZ_l, const real_Y &whlZ_r, const real_Y &vehZ_l, const real_Y &vehZ_r);
};

}	//end of name space
#endif //SUS_INDEPENDENT_TWO_TRACKS_HPP
