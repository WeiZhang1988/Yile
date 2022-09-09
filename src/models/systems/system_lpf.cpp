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
#include "system_lpf.hpp"

void NMSPC::Sys_LPF::push_con_states (d_vec &con_states) {
	m_sptr_lpf->push_con_states(m_lpf_con_states);
	std::copy(m_lpf_con_states.begin(), m_lpf_con_states.end(), con_states.begin());
}

void NMSPC::Sys_LPF::pull_con_states (const d_vec &con_states) {
	std::copy(con_states.begin(), con_states.begin() + Low_Pass_Filter::m_con_states_num, m_lpf_con_states.begin());
	m_sptr_lpf->pull_con_states(m_lpf_con_states);
}

void NMSPC::Sys_LPF::pull_external_inputs (const d_vec &inputs) {
	interface.m_external_inputs = inputs;
}

void NMSPC::Sys_LPF::update_pv() {
	interface.map_lpf_pv();
	m_sptr_lpf->update_pv(interface.m_lpf_pv_inputs,interface.m_lpf_pv_outputs);
}

void NMSPC::Sys_LPF::update_fm() {
	interface.map_lpf_fm();
	m_sptr_lpf->update_fm(interface.m_lpf_fm_inputs,interface.m_lpf_fm_outputs);
}

void NMSPC::Sys_LPF::update_drv() {
	m_sptr_lpf->update_drv(m_lpf_drvs);
	std::copy(m_lpf_drvs.begin(), m_lpf_drvs.end(), m_drvs.begin());
}

void NMSPC::Sys_LPF::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	dxdt = m_drvs;
}
