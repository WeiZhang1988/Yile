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
    static const int m_pv_inputs_num = Sus_Ind_2Tracks::m_pv_inputs_num + Sus_Ind_2Tracks::m_pv_inputs_num;
    static const int m_fm_inputs_num = Sus_Ind_2Tracks::m_fm_inputs_num + Sus_Ind_2Tracks::m_fm_inputs_num;
    static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;
    static const int m_con_states_num = Sus_Ind_2Tracks::m_con_states_num + Sus_Ind_2Tracks::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = Sus_Ind_2Tracks::m_dis_states_num + Sus_Ind_2Tracks::m_dis_states_num;
    static const int m_pv_outputs_num = Sus_Ind_2Tracks::m_pv_outputs_num + Sus_Ind_2Tracks::m_pv_outputs_num;
    static const int m_fm_outputs_num = Sus_Ind_2Tracks::m_fm_outputs_num + Sus_Ind_2Tracks::m_fm_outputs_num;
    static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;

    void add_suses(std::shared_ptr<Sus_Ind_2Tracks> sptr_sus_f, std::shared_ptr<Sus_Ind_2Tracks> sptr_sus_r) {m_sptr_sus_f=sptr_sus_f; m_sptr_sus_r=sptr_sus_r;}

    void push_con_states(d_vec &con_states) {};
	void pull_con_states(const d_vec &con_states) {};

	void pull_pv (const double &Veh_hgt_cg, const double &Veh_r, \
    const double &Strg_str_fl, const double &Tir_Pz_fl, const double &Tir_vz_fl,	const double &Tir_Re_fl, \
	const double &Veh_Pz_fl, const double &Veh_vx_fl, const double &Veh_vy_fl, const double &Veh_vz_fl, \
	const double &Strg_str_fr, const double &Tir_Pz_fr, const double &Tir_vz_fr,	const double &Tir_Re_fr, \
	const double &Veh_Pz_fr, const double &Veh_vx_fr, const double &Veh_vy_fr, const double &Veh_vz_fr, \
    const double &Strg_str_rl, const double &Tir_Pz_rl, const double &Tir_vz_rl,	const double &Tir_Re_rl, \
	const double &Veh_Pz_rl, const double &Veh_vx_rl, const double &Veh_vy_rl, const double &Veh_vz_rl, \
	const double &Strg_str_rr, const double &Tir_Pz_rr, const double &Tir_vz_rr,	const double &Tir_Re_rr, \
	const double &Veh_Pz_rr, const double &Veh_vx_rr, const double &Veh_vy_rr, const double &Veh_vz_rr);
    void push_pv (double &Sus_str_fl, double &Sus_gamma_fl, double &Sus_caster_fl, double &Sus_r_fl, double &Sus_vx_fl, double &Sus_vy_fl, double &Sus_vz_fl, \
	double &Sus_str_fr, double &Sus_gamma_fr, double &Sus_caster_fr, double &Sus_r_fr, double &Sus_vx_fr, double &Sus_vy_fr, double &Sus_vz_fr, \
    double &Sus_str_rl, double &Sus_gamma_rl, double &Sus_caster_rl, double &Sus_r_rl, double &Sus_vx_rl, double &Sus_vy_rl, double &Sus_vz_rl, \
	double &Sus_str_rr, double &Sus_gamma_rr, double &Sus_caster_rr, double &Sus_r_rr, double &Sus_vx_rr, double &Sus_vy_rr, double &Sus_vz_rr);
	void pull_fm(const double &Sus_TirFx_fl, const double &Sus_TirFy_fl, const double &Tir_Mx_fl, const double &Tir_My_fl, const double &Tir_Mz_fl, \
	const double &Sus_TirFx_fr, const double &Sus_TirFy_fr, const double &Tir_Mx_fr, const double &Tir_My_fr, const double &Tir_Mz_fr, \
    const double &Sus_TirFx_rl, const double &Sus_TirFy_rl, const double &Tir_Mx_rl, const double &Tir_My_rl, const double &Tir_Mz_rl, \
	const double &Sus_TirFx_rr, const double &Sus_TirFy_rr, const double &Tir_Mx_rr, const double &Tir_My_rr, const double &Tir_Mz_rr);
    void push_fm (double &Sus_VehFx_fl, double &Sus_VehFy_fl, double &Sus_VehFz_fl, double &Sus_VehMx_fl, double &Sus_VehMy_fl, double &Sus_VehMz_fl, double &Sus_Fz_fl, \
	double &Sus_VehFx_fr, double &Sus_VehFy_fr, double &Sus_VehFz_fr, double &Sus_VehMx_fr, double &Sus_VehMy_fr, double &Sus_VehMz_fr, double &Sus_Fz_fr, \
    double &Sus_VehFx_rl, double &Sus_VehFy_rl, double &Sus_VehFz_rl, double &Sus_VehMx_rl, double &Sus_VehMy_rl, double &Sus_VehMz_rl, double &Sus_Fz_rl, \
	double &Sus_VehFx_rr, double &Sus_VehFy_rr, double &Sus_VehFz_rr, double &Sus_VehMx_rr, double &Sus_VehMy_rr, double &Sus_VehMz_rr, double &Sus_Fz_rr);
	
    void push_drv(d_vec &derivatives) {};

private:
    std::shared_ptr<Sus_Ind_2Tracks> m_sptr_sus_f;
    std::shared_ptr<Sus_Ind_2Tracks> m_sptr_sus_r;
};
} //end of name space
#endif //SUBSYSTEM_SUS_2INDEPENDENT_HPP