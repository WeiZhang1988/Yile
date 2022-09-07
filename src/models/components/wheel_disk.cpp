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
#include "wheel_disk.hpp"

NMSPC::Wheel_Disk::Wheel_Disk (\
const d_vec &params, \
const d_vec &init_states, const bool &init_locked_flag) {
	//initialize parameters
	m_unloaded_radius = params[0];
    m_IYY = params[1];
    m_br = params[2];
    m_disk_abore = params[3];
    m_num_pads = params[4];
    m_Rm = params[5];
    m_mu_kinetic = params[6];
    m_mu_static = params[7];
    //initialize continuous and discrete states
    m_unlocked_omega = init_states[0];
    m_unlocked_omega_pre = init_states[0];
	m_locked_flag = init_locked_flag;
}

void NMSPC::Wheel_Disk::push_con_states (d_vec &con_states) {
	con_states[0] = m_unlocked_omega;
}

void NMSPC::Wheel_Disk::pull_con_states (const d_vec &con_states) {
	m_unlocked_omega = con_states[0];
}

void NMSPC::Wheel_Disk::update_pv(const d_vec &inputs, d_vec &outputs) {
	//pull inputs
	m_Gnd = inputs[0];
	if (m_locked_flag) {
        m_output_omega = 0.0;
        m_unlocked_omega = m_output_omega;
    } else { 
        m_output_omega = m_unlocked_omega;
    } 
    m_rhoz = 0.0;
    m_Re = m_unloaded_radius - m_rhoz;
    m_Pz = m_Gnd;
    m_Vz = 0.0;
    //push outputs
    outputs[0] = m_output_omega;
    outputs[1] = m_Re;
    outputs[2] = m_Pz;
    outputs[3] = m_Vz;
    outputs[4] = m_rhoz;
}

void NMSPC::Wheel_Disk::update_fm (const d_vec &inputs, d_vec &outputs) {
	//pull inputs
	m_Axl_Trq = inputs[0];
	m_Brk_Prs = inputs[1];
	m_Tir_Fx  = inputs[2];
	m_Tir_My  = inputs[3];
	m_Sus_Fz  = inputs[4];
	m_Tir_Fz  = inputs[5];
	//process
	m_Tout = -m_Axl_Trq - m_Tir_My + m_Tir_Fx * saturation(m_Re, 0.0, inf);
    m_Tfmaxk = m_Rm * m_mu_kinetic * saturation(m_Brk_Prs * m_disk_abore * m_disk_abore * m_num_pads * pi / 4.0, eps, inf);
    m_Tfmaxs = m_Tfmaxk * m_mu_static / m_mu_kinetic;
    m_Brk_Trq = m_Tfmaxk;
    //push outputs
    outputs[0] = m_Brk_Trq;
}


void NMSPC::Wheel_Disk::update_drv (d_vec &outputs) {
	//process
	int logic1 = static_cast<int>((abs(m_Tout) <= m_Tfmaxs) && (m_unlocked_omega * m_unlocked_omega_pre <=0));
    int logic2 = static_cast<int>((abs(m_Tout) >= m_Tfmaxs));
    m_locked_flag = static_cast<bool>(m_truth_table[4 * logic1 + 2 * logic2 + static_cast<int>(m_locked_flag)]);
	if (m_locked_flag) {
    	m_unlocked_omega = 0.0;
        m_drv_unlocked_omega = 0.0;
    } else {
        m_drv_unlocked_omega = (m_Tfmaxk * tanh(-4.0 * m_unlocked_omega) - m_Tout - m_br * m_unlocked_omega) / m_IYY;
    }
    //push output
    outputs[0] = m_drv_unlocked_omega;
}
