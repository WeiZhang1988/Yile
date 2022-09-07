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
#include "system_sus_2ind.hpp"

void NMSPC::Sys_Sus_2Ind::push_con_states (d_vec &con_states) {
	m_sptr_subsys_sus_2ind->push_con_states(m_subsys_sus_2ind_con_states);
	std::copy(m_subsys_sus_2ind_con_states.begin(), m_subsys_sus_2ind_con_states.end(), con_states.begin());
}

void NMSPC::Sys_Sus_2Ind::pull_con_states (const d_vec &con_states) {
	std::copy(con_states.begin(), con_states.begin() + Subsys_Sus_2Ind::m_con_states_num, m_subsys_sus_2ind_con_states.begin());
	m_sptr_subsys_sus_2ind->pull_con_states(m_subsys_sus_2ind_con_states);
}

void NMSPC::Sys_Sus_2Ind::pull_external_inputs (const d_vec &inputs) {
	interface.m_external_inputs = inputs;
}

void NMSPC::Sys_Sus_2Ind::update_pv() {
	interface.map_sub_sus_2ind_pv();
	m_sptr_subsys_sus_2ind->update_pv(interface.m_sub_sus_2ind_pv_inputs,interface.m_sub_sus_2ind_pv_outputs);
}

void NMSPC::Sys_Sus_2Ind::update_fm() {
	interface.map_sub_sus_2ind_fm();
	m_sptr_subsys_sus_2ind->update_fm(interface.m_sub_sus_2ind_fm_inputs,interface.m_sub_sus_2ind_fm_outputs);
}

void NMSPC::Sys_Sus_2Ind::update_drv() {
	m_sptr_subsys_sus_2ind->update_drv(m_subsys_sus_2ind_drvs);
	std::copy(m_subsys_sus_2ind_drvs.begin(), m_subsys_sus_2ind_drvs.end(), m_drvs.begin());
}

void NMSPC::Sys_Sus_2Ind::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	dxdt = m_drvs;
}
