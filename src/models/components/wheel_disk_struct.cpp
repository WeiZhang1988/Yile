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

void NMSPC::Wheel_Disk::push_con_states (d_vec &con_states) {
	// con_states[0] = m_unlocked_omega;
    // con_states[1] = m_Tir_Pz;
    // con_states[2] = m_Tir_Vz;
    // con_states[3] = m_Sus_lpf_Fz;
    con_states[0] = m_Wheel_X.unlocked_omega;
    con_states[1] = m_Wheel_X.Tir_Pz; 
    con_states[2] = m_Wheel_X.Tir_Vz;
    con_states[3] = m_Wheel_X.Sus_lpf_Fz;
}

void NMSPC::Wheel_Disk::pull_con_states (const d_vec &con_states) {
    //store previous state
    //m_unlocked_omega_pre = m_unlocked_omega;
    m_Wheel_D.unlocked_omega_pre = m_Wheel_X.unlocked_omega;
    //update current state
    m_Wheel_X.unlocked_omega = con_states[0];
    m_Wheel_X.m_Tir_Pz = con_states[1];
    m_Wheel_X.m_Tir_Vz = con_states[2];
    m_Wheel_X.m_Sus_lpf_Fz = con_states[3];
    
	// m_unlocked_omega = con_states[0];
    // m_Tir_Pz = con_states[1];
    // m_Tir_Vz = con_states[2];
    // m_Sus_lpf_Fz = con_states[3];
}

void NMSPC::Wheel_Disk::update_pv(const d_vec &inputs, d_vec &outputs) {
	//pull inputs
	//m_Gnd_Pz = inputs[0];
    m_Wheel_O.m_Tir_Pz = inputs[0];
    //process
	if (m_locked_flag) {
        m_Tir_omega = 0.0;
        m_unlocked_omega = m_Tir_omega;
    } else { 
        m_Tir_omega = m_unlocked_omega;
    } 
    m_Tir_rhoz = 0.0;
    m_Tir_Re = m_unloaded_radius - m_Tir_rhoz;
    m_Tir_Pz = m_Gnd_Pz;
    m_Tir_Vz = 0.0;
    
    //push outputs
    outputs[0] = m_Tir_omega;
    outputs[1] = m_Tir_Re;
    outputs[2] = m_Tir_Pz;
    outputs[3] = m_Tir_Vz;
    outputs[4] = m_Tir_rhoz;
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
	m_Tout = -m_Axl_Trq - m_Tir_My + m_Tir_Fx * saturation(m_Tir_Re, 0.0, inf);
    m_Tfmaxk = m_Rm * m_mu_kinetic * saturation(m_Brk_Prs * m_disk_abore * m_disk_abore * m_num_pads * pi / 4.0, eps, inf);
    m_Tfmaxs = m_Tfmaxk * m_mu_static / m_mu_kinetic;
    m_Tir_Brk_Trq = m_Tfmaxk;
    //push outputs
    outputs[0] = m_Tir_Brk_Trq;
}


void NMSPC::Wheel_Disk::update_drv (d_vec &outputs) {
	//process
	int logic1 = static_cast<int>((abs(m_Tout) <= m_Tfmaxs) && (m_unlocked_omega * m_unlocked_omega_pre <=0 || abs(m_unlocked_omega) < pi/30.0));
    int logic2 = static_cast<int>((abs(m_Tout) >= m_Tfmaxs));
    m_locked_flag = static_cast<bool>(m_truth_table[4 * logic1 + 2 * logic2 + static_cast<int>(m_locked_flag)]);
	if (m_locked_flag) {
    	m_unlocked_omega = 0.0;
        m_drv_unlocked_omega = 0.0;
    } else {
        m_drv_unlocked_omega = (m_Tfmaxk * tanh(-4.0 * m_unlocked_omega) - m_Tout - m_br * m_unlocked_omega) / m_IYY;
    }
    m_drv_Pz = 0.0;
	m_drv_Vz = 0.0;
    m_drv_Sus_lfp_Fz = (m_Sus_Fz - m_Sus_lpf_Fz) * 200 * pi;

    //push output
    outputs[0] = m_drv_unlocked_omega;
    outputs[1] = m_drv_Pz;
    outputs[2] = m_drv_Vz;
    outputs[3] = m_drv_Sus_lfp_Fz;
}
