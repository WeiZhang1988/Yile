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
#ifndef SUBSYSTEM_SUS_2INDEPENDENT_HPP
#define SUBSYSTEM_SUS_2INDEPENDENT_HPP
#include "components/common.hpp"
#include "components/sus_ind_2tracks.hpp"

namespace NMSPC{
class Subsys_Sus_2Ind {
public:
    static const int m_con_states_num = Sus_Ind_2Tracks::m_con_states_num + Sus_Ind_2Tracks::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = Sus_Ind_2Tracks::m_dis_states_num + Sus_Ind_2Tracks::m_dis_states_num;

	Subsys_Sus_2Ind (real_Y F0z_f=2886.0, real_Y Kz_f=52451.006579283188, real_Y Cz_f=5565.224638019838, \
	real_Y Hmax_f=0.25, real_Y roll_strg_H_slp_f=-0.2269, \
	real_Y toe_f=0.0349, real_Y toe_strg_slp_f=0.01, \
	real_Y caster_f=0.0698, real_Y caster_H_slp_f=-0.2269, real_Y caster_strg_slp_f=0.01, \
	real_Y camber_f=0.0698, real_Y camber_H_slp_f=-0.2269, real_Y camber_strg_slp_f=0.01, \
	real_Y strg_hgt_slp_f=0.1432,\
	real_Y as_R_f=0.2, real_Y as_ntrl_ang_f=0.5236, real_Y as_trsK_f=8e2, \
	bool has_anti_sway_f=true, bool is_strg_f=true, \
	
	real_Y F0z_r=2907.0, real_Y Kz_r=52451.006579283188, real_Y Cz_r=5565.224638019838, \
	real_Y Hmax_r=0.25, real_Y roll_strg_H_slp_r=-0.2269, \
	real_Y toe_r=0.0349, real_Y toe_strg_slp_r=0.01, \
	real_Y caster_r=0.0698, real_Y caster_H_slp_r=-0.2269, real_Y caster_strg_slp_r=0.01, \
	real_Y camber_r=0.0698, real_Y camber_H_slp_r=-0.2269, real_Y camber_strg_slp_r=0.01, \
	real_Y strg_hgt_slp_r=0.1432,\
	real_Y as_R_r=0.2, real_Y as_ntrl_ang_r=0.5236, real_Y as_trsK_r=8e2, \
	bool has_anti_sway_r=false, bool is_strg_r=false)
	{	
		m_sus_f = Sus_Ind_2Tracks(F0z_f, Kz_f, Cz_f, \
		Hmax_f, roll_strg_H_slp_f, \
		toe_f, toe_strg_slp_f, \
		caster_f, caster_H_slp_f, caster_strg_slp_f, \
		camber_f, camber_H_slp_f, camber_strg_slp_f, \
		strg_hgt_slp_f,\
		as_R_f, as_ntrl_ang_f, as_trsK_f, \
		has_anti_sway_f, is_strg_f);
		m_sus_r = Sus_Ind_2Tracks(F0z_r, Kz_r, Cz_r, \
		Hmax_r, roll_strg_H_slp_r, \
		toe_r, toe_strg_slp_r, \
		caster_r, caster_H_slp_r, caster_strg_slp_r, \
		camber_r, camber_H_slp_r, camber_strg_slp_r, \
		strg_hgt_slp_r,\
		as_R_r, as_ntrl_ang_r, as_trsK_r, \
		has_anti_sway_r, is_strg_r);
	}

    void push_con_states(d_vec &con_states) {};
	void pull_con_states(const d_vec &con_states) {};

	void pull_pv (const real_Y &Veh_hgt_cg, const real_Y &Veh_r, \
    const real_Y &Strg_str_fl, const real_Y &Sus_TirPz_fl, const real_Y &Sus_Tirvz_fl,	const real_Y &Tir_Re_fl, \
	const real_Y &Int_Pz_fl, const real_Y &Int_Vz_fl, \
	const real_Y &Veh_vx_fl, const real_Y &Veh_vy_fl, const real_Y &Veh_vz_fl, \
	const real_Y &Strg_str_fr, const real_Y &Sus_TirPz_fr, const real_Y &Sus_Tirvz_fr,	const real_Y &Tir_Re_fr, \
	const real_Y &Int_Pz_fr, const real_Y &Int_Vz_fr, \
	const real_Y &Veh_vx_fr, const real_Y &Veh_vy_fr, const real_Y &Veh_vz_fr, \
    const real_Y &Strg_str_rl, const real_Y &Sus_TirPz_rl, const real_Y &Sus_Tirvz_rl,	const real_Y &Tir_Re_rl, \
	const real_Y &Int_Pz_rl, const real_Y &Int_Vz_rl, \
	const real_Y &Veh_vx_rl, const real_Y &Veh_vy_rl, const real_Y &Veh_vz_rl, \
	const real_Y &Strg_str_rr, const real_Y &Sus_TirPz_rr, const real_Y &Sus_Tirvz_rr,	const real_Y &Tir_Re_rr, \
	const real_Y &Int_Pz_rr, const real_Y &Int_Vz_rr, \
	const real_Y &Veh_vx_rr, const real_Y &Veh_vy_rr, const real_Y &Veh_vz_rr);
    void push_pv (
	real_Y &Sus_str_fl, real_Y &Sus_gamma_fl, real_Y &Sus_caster_fl, real_Y &Sus_r_fl, real_Y &Sus_vx_fl, real_Y &Sus_vy_fl, real_Y &Sus_vz_fl, \
	real_Y &Sus_str_fr, real_Y &Sus_gamma_fr, real_Y &Sus_caster_fr, real_Y &Sus_r_fr, real_Y &Sus_vx_fr, real_Y &Sus_vy_fr, real_Y &Sus_vz_fr, \
    real_Y &Sus_str_rl, real_Y &Sus_gamma_rl, real_Y &Sus_caster_rl, real_Y &Sus_r_rl, real_Y &Sus_vx_rl, real_Y &Sus_vy_rl, real_Y &Sus_vz_rl, \
	real_Y &Sus_str_rr, real_Y &Sus_gamma_rr, real_Y &Sus_caster_rr, real_Y &Sus_r_rr, real_Y &Sus_vx_rr, real_Y &Sus_vy_rr, real_Y &Sus_vz_rr);
	void pull_fm_z ();
	void push_fm_z (real_Y &Sus_VehFz_fl, real_Y &Sus_Fz_fl, real_Y &Sus_VehFz_fr, real_Y &Sus_Fz_fr, real_Y &Sus_VehFz_rl, real_Y &Sus_Fz_rl, real_Y &Sus_VehFz_rr, real_Y &Sus_Fz_rr);
	void pull_fm_o (
	const real_Y &Sus_TirFx_fl, const real_Y &Sus_TirFy_fl, const real_Y &Tir_Mx_fl, const real_Y &Tir_My_fl, const real_Y &Tir_Mz_fl, \
	const real_Y &Sus_TirFx_fr, const real_Y &Sus_TirFy_fr, const real_Y &Tir_Mx_fr, const real_Y &Tir_My_fr, const real_Y &Tir_Mz_fr, \
    const real_Y &Sus_TirFx_rl, const real_Y &Sus_TirFy_rl, const real_Y &Tir_Mx_rl, const real_Y &Tir_My_rl, const real_Y &Tir_Mz_rl, \
	const real_Y &Sus_TirFx_rr, const real_Y &Sus_TirFy_rr, const real_Y &Tir_Mx_rr, const real_Y &Tir_My_rr, const real_Y &Tir_Mz_rr);
	void push_fm_o (
	real_Y &Sus_VehFx_fl, real_Y &Sus_VehFy_fl, real_Y &Sus_VehMx_fl, real_Y &Sus_VehMy_fl, real_Y &Sus_VehMz_fl, \
	real_Y &Sus_VehFx_fr, real_Y &Sus_VehFy_fr, real_Y &Sus_VehMx_fr, real_Y &Sus_VehMy_fr, real_Y &Sus_VehMz_fr, \
    real_Y &Sus_VehFx_rl, real_Y &Sus_VehFy_rl, real_Y &Sus_VehMx_rl, real_Y &Sus_VehMy_rl, real_Y &Sus_VehMz_rl, \
	real_Y &Sus_VehFx_rr, real_Y &Sus_VehFy_rr, real_Y &Sus_VehMx_rr, real_Y &Sus_VehMy_rr, real_Y &Sus_VehMz_rr);
	void pull_fm (
	const real_Y &Sus_TirFx_fl, const real_Y &Sus_TirFy_fl, const real_Y &Tir_Mx_fl, const real_Y &Tir_My_fl, const real_Y &Tir_Mz_fl, \
	const real_Y &Sus_TirFx_fr, const real_Y &Sus_TirFy_fr, const real_Y &Tir_Mx_fr, const real_Y &Tir_My_fr, const real_Y &Tir_Mz_fr, \
    const real_Y &Sus_TirFx_rl, const real_Y &Sus_TirFy_rl, const real_Y &Tir_Mx_rl, const real_Y &Tir_My_rl, const real_Y &Tir_Mz_rl, \
	const real_Y &Sus_TirFx_rr, const real_Y &Sus_TirFy_rr, const real_Y &Tir_Mx_rr, const real_Y &Tir_My_rr, const real_Y &Tir_Mz_rr);
	void push_fm (
	real_Y &Sus_VehFx_fl, real_Y &Sus_VehFy_fl, real_Y &Sus_VehFz_fl, real_Y &Sus_VehMx_fl, real_Y &Sus_VehMy_fl, real_Y &Sus_VehMz_fl, real_Y &Sus_Fz_fl, \
	real_Y &Sus_VehFx_fr, real_Y &Sus_VehFy_fr, real_Y &Sus_VehFz_fr, real_Y &Sus_VehMx_fr, real_Y &Sus_VehMy_fr, real_Y &Sus_VehMz_fr, real_Y &Sus_Fz_fr, \
    real_Y &Sus_VehFx_rl, real_Y &Sus_VehFy_rl, real_Y &Sus_VehFz_rl, real_Y &Sus_VehMx_rl, real_Y &Sus_VehMy_rl, real_Y &Sus_VehMz_rl, real_Y &Sus_Fz_rl, \
	real_Y &Sus_VehFx_rr, real_Y &Sus_VehFy_rr, real_Y &Sus_VehFz_rr, real_Y &Sus_VehMx_rr, real_Y &Sus_VehMy_rr, real_Y &Sus_VehMz_rr, real_Y &Sus_Fz_rr);
	
    void push_drv(d_vec &derivatives) {};

private:
    Sus_Ind_2Tracks m_sus_f, m_sus_r;
	d_vec m_sus_con_states_f = d_vec(Sus_Ind_2Tracks::m_con_states_num, NaN);
    d_vec m_sus_con_states_r = d_vec(Sus_Ind_2Tracks::m_con_states_num, NaN);
};
} //end of name space
#endif //SUBSYSTEM_SUS_2INDEPENDENT_HPP