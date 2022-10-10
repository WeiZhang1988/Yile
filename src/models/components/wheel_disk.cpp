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

NMSPC::Wheel_Disk &NMSPC::Wheel_Disk::operator= (const Wheel_Disk &org) {
    m_unloaded_radius = org.m_unloaded_radius;
    m_IYY = org.m_IYY;
    m_mass = org.m_mass;
    m_br = org.m_br;
    m_disk_abore = org.m_disk_abore;
    m_num_pads = org.m_num_pads;
	m_Rm = org.m_Rm;
    m_mu_kinetic = org.m_mu_kinetic;
    m_mu_static = org.m_mu_static;
	m_unlocked_omega = org.m_unlocked_omega;
    m_unlocked_omega_pre = org.m_unlocked_omega_pre;
	m_Tir_Pz = org.m_Tir_Pz;
    m_Tir_vz = org.m_Tir_vz;
	m_locked_flag = org.m_locked_flag;
    return *this;
}

void NMSPC::Wheel_Disk::push_con_states(d_vec &con_states) {
	con_states[0] = m_unlocked_omega;
    con_states[1] = m_Tir_Pz;
    con_states[2] = m_Tir_vz;
    con_states[3] = m_Sus_lpf_Fz;
}

void NMSPC::Wheel_Disk::pull_con_states(const d_vec &con_states) {
    //store previous state
    m_unlocked_omega_pre = m_unlocked_omega;
    //update current state
	m_unlocked_omega = con_states[0];
    m_Tir_Pz = con_states[1];
    m_Tir_vz = con_states[2];
    m_Sus_lpf_Fz = con_states[3];
}

void NMSPC::Wheel_Disk::pull_pv(const double &Gnd_Pz) {
	//pull inputs
	m_Gnd_Pz = Gnd_Pz;
    //process
	if (m_locked_flag) {
        m_unlocked_omega = 0.0;
    }
    m_Tir_omega = m_unlocked_omega;
    m_Tir_Pz = m_Gnd_Pz;
    m_Tir_vz = 0.0;
    m_Sus_TirPz = -m_Tir_Pz;
    m_Sus_Tirvz = -m_Tir_vz;
    m_Tir_rhoz = 0.0;
    m_Tir_Re = m_unloaded_radius - m_Tir_rhoz;
}

void NMSPC::Wheel_Disk::push_pv(double &Tir_omega, double &Sus_TirPz, double &Sus_Tirvz, double &Tir_Pz, double &Tir_vz, double &Tir_rhoz, double &Tir_Re) {
    Tir_omega = m_Tir_omega;
    Tir_Pz = m_Tir_Pz;
    Tir_vz = m_Tir_vz;
    Sus_TirPz = m_Sus_TirPz;
    Sus_Tirvz = m_Sus_Tirvz;
    Tir_rhoz = m_Tir_rhoz;
    Tir_Re = m_Tir_Re;
}

void NMSPC::Wheel_Disk::pull_fm(const double &Axl_Trq, const double &Brk_Prs, const double &Tir_Fx, const double &Tir_My, const double &Tir_Fz, const double &Sus_Fz) {
	//pull inputs
	m_Axl_Trq = Axl_Trq;
	m_Brk_Prs = Brk_Prs;
	m_Tir_Fx  = Tir_Fx;
	m_Tir_My  = Tir_My;
	m_Tir_Fz  = Tir_Fz;
	m_Sus_Fz  = Sus_Fz;
	//process
	m_Tout = -m_Axl_Trq - m_Tir_My + m_Tir_Fx * saturation(m_Tir_Re, 0.0, inf);
    m_Tfmaxk = m_Rm * m_mu_kinetic * saturation(m_Brk_Prs * m_disk_abore * m_disk_abore * m_num_pads * pi / 4.0, eps, inf);
    m_Tfmaxs = m_Tfmaxk * m_mu_static / m_mu_kinetic;
    m_Tir_Brk_Trq = m_Tfmaxk;
}

void NMSPC::Wheel_Disk::push_fm(double &Brk_Trq) {
    Brk_Trq = m_Tir_Brk_Trq;
}


void NMSPC::Wheel_Disk::push_drv(d_vec &derivatives) {
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
    m_drv_Tir_Pz = m_Tir_vz;
	m_drv_Tir_vz = (m_Tir_Fz - saturation(m_Sus_lpf_Fz, -10.0*9.81*2000, inf)) / m_mass;
    m_drv_Sus_lfp_Fz = (m_Sus_Fz - m_Sus_lpf_Fz) * m_lpf_wc;

    //push output
    derivatives[0] = m_drv_unlocked_omega;
    derivatives[1] = m_drv_Tir_Pz;
    derivatives[2] = m_drv_Tir_vz;
    derivatives[3] = m_drv_Sus_lfp_Fz;
}
