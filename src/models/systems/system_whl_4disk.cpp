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
#include "system_whl_4disk.hpp"

void NMSPC::Sys_Whl_4Disk::push_con_states (d_vec &con_states) {
	m_sptr_subsys_whl_4disk->push_con_states(m_subsys_whl_4disk_con_states);
	std::copy(m_subsys_whl_4disk_con_states.begin(), m_subsys_whl_4disk_con_states.end(), con_states.begin());
}

void NMSPC::Sys_Whl_4Disk::pull_con_states (const d_vec &con_states) {
	std::copy(con_states.begin(), con_states.begin() + Subsys_Wheel_4Disk::m_con_states_num, m_subsys_whl_4disk_con_states.begin());
	m_sptr_subsys_whl_4disk->pull_con_states(m_subsys_whl_4disk_con_states);
}

void NMSPC::Sys_Whl_4Disk::update_pv() {
	m_sptr_subsys_whl_4disk->pull_pv(m_sptr_interface->m_Gnd_Pz_fl, m_sptr_interface->m_Gnd_Pz_fr, m_sptr_interface->m_Gnd_Pz_rl, m_sptr_interface->m_Gnd_Pz_rr);
	m_sptr_subsys_whl_4disk->push_pv(m_sptr_interface->m_Tir_omega_fl, m_sptr_interface->m_Sus_TirPz_fl, m_sptr_interface->m_Sus_Tirvz_fl, m_sptr_interface->m_Tir_Pz_fl, m_sptr_interface->m_Tir_vz_fl, m_sptr_interface->m_Tir_rhoz_fl, m_sptr_interface->m_Tir_Re_fl, \
    m_sptr_interface->m_Tir_omega_fr, m_sptr_interface->m_Sus_TirPz_fr, m_sptr_interface->m_Sus_Tirvz_fr, m_sptr_interface->m_Tir_Pz_fr, m_sptr_interface->m_Tir_vz_fr, m_sptr_interface->m_Tir_rhoz_fr, m_sptr_interface->m_Tir_Re_fr, \
    m_sptr_interface->m_Tir_omega_rl, m_sptr_interface->m_Sus_TirPz_rl, m_sptr_interface->m_Sus_Tirvz_rl, m_sptr_interface->m_Tir_Pz_rl, m_sptr_interface->m_Tir_vz_rl, m_sptr_interface->m_Tir_rhoz_rl, m_sptr_interface->m_Tir_Re_rl, \
    m_sptr_interface->m_Tir_omega_rr, m_sptr_interface->m_Sus_TirPz_rr, m_sptr_interface->m_Sus_Tirvz_rr, m_sptr_interface->m_Tir_Pz_rr, m_sptr_interface->m_Tir_vz_rr, m_sptr_interface->m_Tir_rhoz_rr, m_sptr_interface->m_Tir_Re_rr);
}

void NMSPC::Sys_Whl_4Disk::update_fm() {
	m_sptr_subsys_whl_4disk->pull_fm(m_sptr_interface->m_Axl_Trq_fl, m_sptr_interface->m_Brk_Prs_fl, m_sptr_interface->m_Tir_Fx_fl, m_sptr_interface->m_Tir_My_fl, m_sptr_interface->m_Tir_Fz_fl, m_sptr_interface->m_Sus_Fz_fl,\
    m_sptr_interface->m_Axl_Trq_fr, m_sptr_interface->m_Brk_Prs_fr, m_sptr_interface->m_Tir_Fx_fr, m_sptr_interface->m_Tir_My_fr, m_sptr_interface->m_Tir_Fz_fr, m_sptr_interface->m_Sus_Fz_fr,\
    m_sptr_interface->m_Axl_Trq_rl, m_sptr_interface->m_Brk_Prs_rl, m_sptr_interface->m_Tir_Fx_rl, m_sptr_interface->m_Tir_My_rl, m_sptr_interface->m_Tir_Fz_rl, m_sptr_interface->m_Sus_Fz_rl,\
    m_sptr_interface->m_Axl_Trq_rr, m_sptr_interface->m_Brk_Prs_rr, m_sptr_interface->m_Tir_Fx_rr, m_sptr_interface->m_Tir_My_rr, m_sptr_interface->m_Tir_Fz_rr, m_sptr_interface->m_Sus_Fz_rr);
	m_sptr_subsys_whl_4disk->push_fm(m_sptr_interface->m_Brk_Trq_fl, m_sptr_interface->m_Brk_Trq_fr, m_sptr_interface->m_Brk_Trq_rl, m_sptr_interface->m_Brk_Trq_rr);
}

void NMSPC::Sys_Whl_4Disk::update_drv() {
	m_sptr_subsys_whl_4disk->push_drv(m_subsys_whl_4disk_drvs);
	std::copy(m_subsys_whl_4disk_drvs.begin(), m_subsys_whl_4disk_drvs.end(), m_drvs.begin());
}

void NMSPC::Sys_Whl_4Disk::store_data() {
	m_sptr_store->push_back(d_vec{
		m_sptr_interface->m_Tir_omega_fl, 
		m_sptr_interface->m_Sus_TirPz_fl, 
		m_sptr_interface->m_Sus_Tirvz_fl,
		m_sptr_interface->m_Tir_Pz_fl, 
		m_sptr_interface->m_Tir_vz_fl, 
		m_sptr_interface->m_Tir_rhoz_fl, 
		m_sptr_interface->m_Tir_Re_fl, 
		m_sptr_interface->m_Brk_Trq_fl,

		m_sptr_interface->m_Tir_omega_fr, 
		m_sptr_interface->m_Sus_TirPz_fr, 
		m_sptr_interface->m_Sus_Tirvz_fr,
		m_sptr_interface->m_Tir_Pz_fr, 
		m_sptr_interface->m_Tir_vz_fr, 
		m_sptr_interface->m_Tir_rhoz_fr, 
		m_sptr_interface->m_Tir_Re_fr, 
		m_sptr_interface->m_Brk_Trq_fr,

		m_sptr_interface->m_Tir_omega_rl, 
		m_sptr_interface->m_Sus_TirPz_rl, 
		m_sptr_interface->m_Sus_Tirvz_rl,
		m_sptr_interface->m_Tir_Pz_rl, 
		m_sptr_interface->m_Tir_vz_rl, 
		m_sptr_interface->m_Tir_rhoz_rl, 
		m_sptr_interface->m_Tir_Re_rl, 
		m_sptr_interface->m_Brk_Trq_rl,

		m_sptr_interface->m_Tir_omega_rr, 
		m_sptr_interface->m_Sus_TirPz_rr, 
		m_sptr_interface->m_Sus_Tirvz_rr,
		m_sptr_interface->m_Tir_Pz_rr, 
		m_sptr_interface->m_Tir_vz_rr, 
		m_sptr_interface->m_Tir_rhoz_rr, 
		m_sptr_interface->m_Tir_Re_rr, 
		m_sptr_interface->m_Brk_Trq_rr
		});
}

void NMSPC::Sys_Whl_4Disk::operator() (const d_vec &x, d_vec &dxdt, const real_Y &t) {
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	store_data();
	dxdt = m_drvs;
}