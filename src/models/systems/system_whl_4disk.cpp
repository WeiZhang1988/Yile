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

void NMSPC::Sys_Whl_4Disk::pull_external_inputs (const d_vec &inputs) {
	m_sptr_interface->m_external_inputs = inputs;
}

void NMSPC::Sys_Whl_4Disk::update_pv() {
	m_sptr_interface->map_sub_whl_4disk_pv();
	m_sptr_subsys_whl_4disk->update_pv(m_sptr_interface->m_sub_whl_4disk_pv_inputs,m_sptr_interface->m_sub_whl_4disk_pv_outputs);
}

void NMSPC::Sys_Whl_4Disk::update_fm() {
	m_sptr_interface->map_sub_whl_4disk_fm();
	m_sptr_subsys_whl_4disk->update_fm(m_sptr_interface->m_sub_whl_4disk_fm_inputs,m_sptr_interface->m_sub_whl_4disk_fm_outputs);
}

void NMSPC::Sys_Whl_4Disk::update_drv() {
	m_sptr_subsys_whl_4disk->update_drv(m_subsys_whl_4disk_drvs);
	std::copy(m_subsys_whl_4disk_drvs.begin(), m_subsys_whl_4disk_drvs.end(), m_drvs.begin());
}

void NMSPC::Sys_Whl_4Disk::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	dxdt = m_drvs;
}
