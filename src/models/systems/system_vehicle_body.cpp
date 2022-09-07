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
#include "system_vehicle_body.hpp"

void NMSPC::Sys_Vehicle_Body::push_con_states (d_vec &con_states) {
	m_sptr_vhl_bdy->push_con_states(m_vhl_bdy_con_states);
	std::copy(m_vhl_bdy_con_states.begin(), m_vhl_bdy_con_states.end(), con_states.begin());
}

void NMSPC::Sys_Vehicle_Body::pull_con_states (const d_vec &con_states) {
	std::copy(con_states.begin(), con_states.begin() + Vehicle_Body::m_con_states_num, m_vhl_bdy_con_states.begin());
	m_sptr_vhl_bdy->pull_con_states(m_vhl_bdy_con_states);
}

void NMSPC::Sys_Vehicle_Body::pull_external_inputs (const d_vec &inputs) {
	interface.m_external_inputs = inputs;
}

void NMSPC::Sys_Vehicle_Body::update_pv() {
	interface.map_vhl_bdy_pv();
	m_sptr_vhl_bdy->update_pv(interface.m_vhl_bdy_pv_inputs,interface.m_vhl_bdy_pv_outputs);
}

void NMSPC::Sys_Vehicle_Body::update_fm() {
	interface.map_vhl_bdy_fm();
	m_sptr_vhl_bdy->update_fm(interface.m_vhl_bdy_fm_inputs,interface.m_vhl_bdy_fm_outputs);
}

void NMSPC::Sys_Vehicle_Body::update_drv() {
	m_sptr_vhl_bdy->update_drv(m_vhl_bdy_drvs);
	std::copy(m_vhl_bdy_drvs.begin(), m_vhl_bdy_drvs.end(), m_drvs.begin());
}

void NMSPC::Sys_Vehicle_Body::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	dxdt = m_drvs;	
}
