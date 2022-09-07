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
#include "system_whl_disk_tir_fiala.hpp"

void NMSPC::Sys_Whl_Disk_Tir_Fiala::push_con_states (d_vec &con_states) {
	m_sptr_whl->push_con_states(m_whl_con_states);
	std::copy(m_whl_con_states.begin(), m_whl_con_states.end(), con_states.begin());
	m_sptr_tir->push_con_states(m_tir_con_states);
	std::copy(m_tir_con_states.begin(), m_tir_con_states.end(), con_states.begin() + Wheel_Disk::m_con_states_num);
}

void NMSPC::Sys_Whl_Disk_Tir_Fiala::pull_con_states (const d_vec &con_states) {
	std::copy(con_states.begin(), con_states.begin() + Wheel_Disk::m_con_states_num, m_whl_con_states.begin());
	m_sptr_whl->pull_con_states(m_whl_con_states);
	std::copy(con_states.begin() + Wheel_Disk::m_con_states_num, con_states.end(), m_tir_con_states.begin());
	m_sptr_tir->pull_con_states(m_tir_con_states);
}

void NMSPC::Sys_Whl_Disk_Tir_Fiala::pull_external_inputs (const d_vec &inputs) {
	interface.m_external_inputs = inputs;
}

void NMSPC::Sys_Whl_Disk_Tir_Fiala::update_pv() {
	interface.map_whl_pv();
	m_sptr_whl->update_pv(interface.m_whl_pv_inputs,interface.m_whl_pv_outputs);
	interface.map_tir_pv();
	m_sptr_tir->update_pv(interface.m_tir_pv_inputs,interface.m_tir_pv_outputs);
}

void NMSPC::Sys_Whl_Disk_Tir_Fiala::update_fm() {
	interface.map_tir_fm();
	m_sptr_tir->update_fm(interface.m_tir_fm_inputs,interface.m_tir_fm_outputs);
	interface.map_whl_fm();
	m_sptr_whl->update_fm(interface.m_whl_fm_inputs,interface.m_whl_fm_outputs);
}

void NMSPC::Sys_Whl_Disk_Tir_Fiala::update_drv() {
	m_sptr_whl->update_drv(m_whl_drv_states);
	std::copy(m_whl_drv_states.begin(), m_whl_drv_states.end(), m_drvs.begin());
	m_sptr_tir->update_drv(m_tir_drv_states);
	std::copy(m_tir_drv_states.begin(), m_tir_drv_states.end(), m_drvs.begin() + Wheel_Disk::m_derivatives_num);
}

void NMSPC::Sys_Whl_Disk_Tir_Fiala::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	dxdt = m_drvs;
}
