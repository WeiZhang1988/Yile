// =============================================================================
// PROJECT YILE - 
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of this source code is governed by a GPL-3.0 license that can be found
// in the LICENSE file
//
// Author of this file	Wei ZHANG wei_zhang_1988@outlook.com,ChangMeng Hou 945881625@qq.com
//
// =============================================================================
#include "system_sus_2ind.hpp"

void NMSPC::Sys_Sus_2Ind::push_con_states (d_vec &con_states) {
	m_sptr_subsys_sus_2ind->push_con_states(m_subsys_sus_2ind_con_states);
	std::copy(m_subsys_sus_2ind_con_states.begin(), m_subsys_sus_2ind_con_states.end(), con_states.begin());
}

void NMSPC::Sys_Sus_2Ind::pull_con_states (const d_vec &con_states) {
	std::copy(con_states.begin(), con_states.begin() + Subsys_Sus_2Ind::m_con_states_num, m_subsys_sus_2ind_con_states.begin());
	m_sptr_subsys_sus_2ind->pull_con_states(m_subsys_sus_2ind_con_states);
}

void NMSPC::Sys_Sus_2Ind::update_pv() {
	m_sptr_subsys_sus_2ind->pull_pv(m_sptr_interface->m_Veh_hgt_cg, m_sptr_interface->m_Veh_r, \
    m_sptr_interface->m_Strg_str_fl, m_sptr_interface->m_Sus_TirPz_fl, m_sptr_interface->m_Sus_Tirvz_fl, m_sptr_interface->m_Tir_Re_fl, \
	m_sptr_interface->m_Int_Pz_fl, m_sptr_interface->m_Int_Vz_fl, m_sptr_interface->m_Veh_vx_fl, m_sptr_interface->m_Veh_vy_fl, m_sptr_interface->m_Veh_vz_fl, \
	m_sptr_interface->m_Strg_str_fr, m_sptr_interface->m_Sus_TirPz_fr, m_sptr_interface->m_Sus_Tirvz_fr, m_sptr_interface->m_Tir_Re_fr, \
	m_sptr_interface->m_Int_Pz_fr, m_sptr_interface->m_Int_Vz_fr, m_sptr_interface->m_Veh_vx_fr, m_sptr_interface->m_Veh_vy_fr, m_sptr_interface->m_Veh_vz_fr, \
    m_sptr_interface->m_Strg_str_rl, m_sptr_interface->m_Sus_TirPz_rl, m_sptr_interface->m_Sus_Tirvz_rl, m_sptr_interface->m_Tir_Re_rl, \
	m_sptr_interface->m_Int_Pz_rl, m_sptr_interface->m_Int_Vz_rl, m_sptr_interface->m_Veh_vx_rl, m_sptr_interface->m_Veh_vy_rl, m_sptr_interface->m_Veh_vz_rl, \
	m_sptr_interface->m_Strg_str_rr, m_sptr_interface->m_Sus_TirPz_rr, m_sptr_interface->m_Sus_Tirvz_rr, m_sptr_interface->m_Tir_Re_rr, \
	m_sptr_interface->m_Int_Pz_rr, m_sptr_interface->m_Int_Vz_rr, m_sptr_interface->m_Veh_vx_rr, m_sptr_interface->m_Veh_vy_rr, m_sptr_interface->m_Veh_vz_rr);
	m_sptr_subsys_sus_2ind->push_pv(m_sptr_interface->m_Sus_str_fl, m_sptr_interface->m_Sus_gamma_fl, m_sptr_interface->m_Sus_caster_fl, m_sptr_interface->m_Sus_r_fl, m_sptr_interface->m_Sus_vx_fl, m_sptr_interface->m_Sus_vy_fl, m_sptr_interface->m_Sus_vz_fl, \
	m_sptr_interface->m_Sus_str_fr, m_sptr_interface->m_Sus_gamma_fr, m_sptr_interface->m_Sus_caster_fr, m_sptr_interface->m_Sus_r_fr, m_sptr_interface->m_Sus_vx_fr, m_sptr_interface->m_Sus_vy_fr, m_sptr_interface->m_Sus_vz_fr, \
    m_sptr_interface->m_Sus_str_rl, m_sptr_interface->m_Sus_gamma_rl, m_sptr_interface->m_Sus_caster_rl, m_sptr_interface->m_Sus_r_rl, m_sptr_interface->m_Sus_vx_rl, m_sptr_interface->m_Sus_vy_rl, m_sptr_interface->m_Sus_vz_rl, \
	m_sptr_interface->m_Sus_str_rr, m_sptr_interface->m_Sus_gamma_rr, m_sptr_interface->m_Sus_caster_rr, m_sptr_interface->m_Sus_r_rr, m_sptr_interface->m_Sus_vx_rr, m_sptr_interface->m_Sus_vy_rr, m_sptr_interface->m_Sus_vz_rr);
}

void NMSPC::Sys_Sus_2Ind::update_fm() {
	m_sptr_subsys_sus_2ind->pull_fm(m_sptr_interface->m_Sus_TirFx_fl, m_sptr_interface->m_Sus_TirFy_fl, m_sptr_interface->m_Tir_Mx_fl, m_sptr_interface->m_Tir_My_fl, m_sptr_interface->m_Tir_Mz_fl, \
	m_sptr_interface->m_Sus_TirFx_fr, m_sptr_interface->m_Sus_TirFy_fr, m_sptr_interface->m_Tir_Mx_fr, m_sptr_interface->m_Tir_My_fr, m_sptr_interface->m_Tir_Mz_fr, \
    m_sptr_interface->m_Sus_TirFx_rl, m_sptr_interface->m_Sus_TirFy_rl, m_sptr_interface->m_Tir_Mx_rl, m_sptr_interface->m_Tir_My_rl, m_sptr_interface->m_Tir_Mz_rl, \
	m_sptr_interface->m_Sus_TirFx_rr, m_sptr_interface->m_Sus_TirFy_rr, m_sptr_interface->m_Tir_Mx_rr, m_sptr_interface->m_Tir_My_rr, m_sptr_interface->m_Tir_Mz_rr);
	m_sptr_subsys_sus_2ind->push_fm(m_sptr_interface->m_Sus_VehFx_fl, m_sptr_interface->m_Sus_VehFy_fl, m_sptr_interface->m_Sus_VehFz_fl, m_sptr_interface->m_Sus_VehMx_fl, m_sptr_interface->m_Sus_VehMy_fl, m_sptr_interface->m_Sus_VehMz_fl, m_sptr_interface->m_Sus_Fz_fl, \
	m_sptr_interface->m_Sus_VehFx_fr, m_sptr_interface->m_Sus_VehFy_fr, m_sptr_interface->m_Sus_VehFz_fr, m_sptr_interface->m_Sus_VehMx_fr, m_sptr_interface->m_Sus_VehMy_fr, m_sptr_interface->m_Sus_VehMz_fr, m_sptr_interface->m_Sus_Fz_fr, \
    m_sptr_interface->m_Sus_VehFx_rl, m_sptr_interface->m_Sus_VehFy_rl, m_sptr_interface->m_Sus_VehFz_rl, m_sptr_interface->m_Sus_VehMx_rl, m_sptr_interface->m_Sus_VehMy_rl, m_sptr_interface->m_Sus_VehMz_rl, m_sptr_interface->m_Sus_Fz_rl, \
	m_sptr_interface->m_Sus_VehFx_rr, m_sptr_interface->m_Sus_VehFy_rr, m_sptr_interface->m_Sus_VehFz_rr, m_sptr_interface->m_Sus_VehMx_rr, m_sptr_interface->m_Sus_VehMy_rr, m_sptr_interface->m_Sus_VehMz_rr, m_sptr_interface->m_Sus_Fz_rr);
}

void NMSPC::Sys_Sus_2Ind::update_drv() {
	m_sptr_subsys_sus_2ind->push_drv(m_subsys_sus_2ind_drvs);
}

void NMSPC::Sys_Sus_2Ind::store_data() {
	m_sptr_store->push_back(d_vec{	
		m_sptr_interface->m_Sus_VehFx_fl, 
		m_sptr_interface->m_Sus_VehFy_fl, // Fx and Fy are useless
		m_sptr_interface->m_Sus_VehFz_fl, 
		m_sptr_interface->m_Sus_VehMx_fl, 
		m_sptr_interface->m_Sus_VehMy_fl, 
		m_sptr_interface->m_Sus_VehMz_fl, 
		m_sptr_interface->m_Sus_Fz_fl ,
		m_sptr_interface->m_Sus_gamma_fl,
		m_sptr_interface->m_Sus_caster_fl,
		m_sptr_interface->m_Sus_str_fl,
		
		m_sptr_interface->m_Sus_VehFx_fr, 
		m_sptr_interface->m_Sus_VehFy_fr, // Fx and Fy are useless
		m_sptr_interface->m_Sus_VehFz_fr, 
		m_sptr_interface->m_Sus_VehMx_fr, 
		m_sptr_interface->m_Sus_VehMy_fr, 
		m_sptr_interface->m_Sus_VehMz_fr, 
		m_sptr_interface->m_Sus_Fz_fr ,
		m_sptr_interface->m_Sus_gamma_fr,
		m_sptr_interface->m_Sus_caster_fr,
		m_sptr_interface->m_Sus_str_fr,

		m_sptr_interface->m_Sus_VehFx_rl, 
		m_sptr_interface->m_Sus_VehFy_rl, // Fx and Fy are useless
		m_sptr_interface->m_Sus_VehFz_rl, 
		m_sptr_interface->m_Sus_VehMx_rl, 
		m_sptr_interface->m_Sus_VehMy_rl, 
		m_sptr_interface->m_Sus_VehMz_rl, 
		m_sptr_interface->m_Sus_Fz_rl,
		m_sptr_interface->m_Sus_gamma_rl,
		m_sptr_interface->m_Sus_caster_rl,
		m_sptr_interface->m_Sus_str_rl,

		m_sptr_interface->m_Sus_VehFx_rr, 
		m_sptr_interface->m_Sus_VehFy_rr, // Fx and Fy are useless
		m_sptr_interface->m_Sus_VehFz_rr, 
		m_sptr_interface->m_Sus_VehMx_rr, 
		m_sptr_interface->m_Sus_VehMy_rr, 
		m_sptr_interface->m_Sus_VehMz_rr, 
		m_sptr_interface->m_Sus_Fz_rr ,
		m_sptr_interface->m_Sus_gamma_rr,
		m_sptr_interface->m_Sus_caster_rr,
		m_sptr_interface->m_Sus_str_rr}
		);
}

void NMSPC::Sys_Sus_2Ind::operator() (const d_vec &x, d_vec &dxdt, const real_Y &t) {
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	store_data();
	dxdt = m_drvs;
}
