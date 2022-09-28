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

	Subsys_Sus_2Ind (double F0z_f=2886.0, double Kz_f=52451.006579283188, double Cz_f=5565.226438019838, \
	double Hmax_f=0.25, double roll_strg_H_slp_f=-0.2269, \
	double toe_f=0.0349, double toe_strg_slp_f=0.01, \
	double caster_f=0.0698, double caster_H_slp_f=-0.2269, double caster_strg_slp_f=0.01, \
	double camber_f=0.0698, double camber_H_slp_f=-0.2269, double camber_strg_slp_f=0.01, \
	double strg_hgt_slp_f=0.1432,\
	double as_R_f=0.2, double as_ntrl_ang_f=0.5236, double as_trsK_f=8e2, \
	bool has_anti_sway_f=true, bool is_strg_f=true, \
	
	double F0z_r=2907.0, double Kz_r=52451.006579283188, double Cz_r=5565.226438019838, \
	double Hmax_r=0.25, double roll_strg_H_slp_r=-0.2269, \
	double toe_r=0.0349, double toe_strg_slp_r=0.01, \
	double caster_r=0.0698, double caster_H_slp_r=-0.2269, double caster_strg_slp_r=0.01, \
	double camber_r=0.0698, double camber_H_slp_r=-0.2269, double camber_strg_slp_r=0.01, \
	double strg_hgt_slp_r=0.1432,\
	double as_R_r=0.2, double as_ntrl_ang_r=0.5236, double as_trsK_r=8e2, \
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

	void pull_pv (const double &Veh_hgt_cg, const double &Veh_r, \
    const double &Strg_str_fl, const double &Sus_TirPz_fl, const double &Sus_Tirvz_fl,	const double &Tir_Re_fl, \
	const double &Veh_Pz_fl, const double &Veh_vx_fl, const double &Veh_vy_fl, const double &Veh_vz_fl, \
	const double &Strg_str_fr, const double &Sus_TirPz_fr, const double &Sus_Tirvz_fr,	const double &Tir_Re_fr, \
	const double &Veh_Pz_fr, const double &Veh_vx_fr, const double &Veh_vy_fr, const double &Veh_vz_fr, \
    const double &Strg_str_rl, const double &Sus_TirPz_rl, const double &Sus_Tirvz_rl,	const double &Tir_Re_rl, \
	const double &Veh_Pz_rl, const double &Veh_vx_rl, const double &Veh_vy_rl, const double &Veh_vz_rl, \
	const double &Strg_str_rr, const double &Sus_TirPz_rr, const double &Sus_Tirvz_rr,	const double &Tir_Re_rr, \
	const double &Veh_Pz_rr, const double &Veh_vx_rr, const double &Veh_vy_rr, const double &Veh_vz_rr);
    void push_pv (double &Sus_str_fl, double &Sus_gamma_fl, double &Sus_caster_fl, double &Sus_r_fl, double &Sus_vx_fl, double &Sus_vy_fl, double &Sus_vz_fl, \
	double &Sus_str_fr, double &Sus_gamma_fr, double &Sus_caster_fr, double &Sus_r_fr, double &Sus_vx_fr, double &Sus_vy_fr, double &Sus_vz_fr, \
    double &Sus_str_rl, double &Sus_gamma_rl, double &Sus_caster_rl, double &Sus_r_rl, double &Sus_vx_rl, double &Sus_vy_rl, double &Sus_vz_rl, \
	double &Sus_str_rr, double &Sus_gamma_rr, double &Sus_caster_rr, double &Sus_r_rr, double &Sus_vx_rr, double &Sus_vy_rr, double &Sus_vz_rr);
	void pull_fm_z ();
	void push_fm_z (double &Sus_VehFz_fl, double &Sus_Fz_fl, double &Sus_VehFz_fr, double &Sus_Fz_fr, double &Sus_VehFz_rl, double &Sus_Fz_rl, double &Sus_VehFz_rr, double &Sus_Fz_rr);
	void pull_fm_o (const double &Sus_TirFx_fl, const double &Sus_TirFy_fl, const double &Tir_Mx_fl, const double &Tir_My_fl, const double &Tir_Mz_fl, \
	const double &Sus_TirFx_fr, const double &Sus_TirFy_fr, const double &Tir_Mx_fr, const double &Tir_My_fr, const double &Tir_Mz_fr, \
    const double &Sus_TirFx_rl, const double &Sus_TirFy_rl, const double &Tir_Mx_rl, const double &Tir_My_rl, const double &Tir_Mz_rl, \
	const double &Sus_TirFx_rr, const double &Sus_TirFy_rr, const double &Tir_Mx_rr, const double &Tir_My_rr, const double &Tir_Mz_rr);
	void push_fm_o (double &Sus_VehFx_fl, double &Sus_VehFy_fl, double &Sus_VehMx_fl, double &Sus_VehMy_fl, double &Sus_VehMz_fl, \
	double &Sus_VehFx_fr, double &Sus_VehFy_fr, double &Sus_VehMx_fr, double &Sus_VehMy_fr, double &Sus_VehMz_fr, \
    double &Sus_VehFx_rl, double &Sus_VehFy_rl, double &Sus_VehMx_rl, double &Sus_VehMy_rl, double &Sus_VehMz_rl, \
	double &Sus_VehFx_rr, double &Sus_VehFy_rr, double &Sus_VehMx_rr, double &Sus_VehMy_rr, double &Sus_VehMz_rr);
	void pull_fm (const double &Sus_TirFx_fl, const double &Sus_TirFy_fl, const double &Tir_Mx_fl, const double &Tir_My_fl, const double &Tir_Mz_fl, \
	const double &Sus_TirFx_fr, const double &Sus_TirFy_fr, const double &Tir_Mx_fr, const double &Tir_My_fr, const double &Tir_Mz_fr, \
    const double &Sus_TirFx_rl, const double &Sus_TirFy_rl, const double &Tir_Mx_rl, const double &Tir_My_rl, const double &Tir_Mz_rl, \
	const double &Sus_TirFx_rr, const double &Sus_TirFy_rr, const double &Tir_Mx_rr, const double &Tir_My_rr, const double &Tir_Mz_rr);
	void push_fm (double &Sus_VehFx_fl, double &Sus_VehFy_fl, double &Sus_VehFz_fl, double &Sus_VehMx_fl, double &Sus_VehMy_fl, double &Sus_VehMz_fl, double &Sus_Fz_fl, \
	double &Sus_VehFx_fr, double &Sus_VehFy_fr, double &Sus_VehFz_fr, double &Sus_VehMx_fr, double &Sus_VehMy_fr, double &Sus_VehMz_fr, double &Sus_Fz_fr, \
    double &Sus_VehFx_rl, double &Sus_VehFy_rl, double &Sus_VehFz_rl, double &Sus_VehMx_rl, double &Sus_VehMy_rl, double &Sus_VehMz_rl, double &Sus_Fz_rl, \
	double &Sus_VehFx_rr, double &Sus_VehFy_rr, double &Sus_VehFz_rr, double &Sus_VehMx_rr, double &Sus_VehMy_rr, double &Sus_VehMz_rr, double &Sus_Fz_rr);
	
    void push_drv(d_vec &derivatives) {};

private:
    Sus_Ind_2Tracks m_sus_f, m_sus_r;
	d_vec m_sus_con_states_f = d_vec(Sus_Ind_2Tracks::m_con_states_num, NaN);
    d_vec m_sus_con_states_r = d_vec(Sus_Ind_2Tracks::m_con_states_num, NaN);
};
} //end of name space
#endif //SUBSYSTEM_SUS_2INDEPENDENT_HPP