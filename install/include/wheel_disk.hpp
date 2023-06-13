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
#ifndef WHEEL_DISK_HPP
#define WHEEL_DISK_HPP
#include "common.hpp"

namespace NMSPC{
class Wheel_Disk {
public:
	static const int m_con_states_num  = 4;									//amount of continuous states;								
	static const int m_derivatives_num = m_con_states_num;					//amount of derivatives;
	static const int m_dis_states_num  = 1;									//amount of discrete states;

	Wheel_Disk (\
	real_Y unloaded_radius=0.309384029954441, real_Y IYY=0.740633832792491, real_Y mass = 5.0,\
	real_Y br=1e-3, real_Y disk_abore=0.05, real_Y num_pads=2.0, \
	real_Y Rm=0.177, real_Y mu_kinetic=0.2, real_Y mu_static=0.3, \
	real_Y init_omega=0.0, real_Y init_Pz=0.0, real_Y init_vz=0.0, \
	bool init_locked_flag=false, \
	bool init_locked_state=false) :
	m_unloaded_radius(unloaded_radius),m_IYY(IYY), m_mass(mass), \
	m_br(br), m_disk_abore(disk_abore), m_num_pads(num_pads), \
	m_Rm(Rm), m_mu_kinetic(mu_kinetic), m_mu_static(mu_static), \
	m_unlocked_omega(init_omega), m_unlocked_omega_pre(init_omega), \
	m_Tir_Pz(init_Pz), m_Tir_vz(init_vz), \
	m_locked_flag(init_locked_flag), \
	m_locked_state(init_locked_state) {};

	Wheel_Disk &operator= (const Wheel_Disk &org);

	void push_con_states (d_vec &con_states);
	void pull_con_states (const d_vec &con_states);

	void pull_pv (const real_Y &Gnd_Pz);
	void push_pv (real_Y &Tir_omega, real_Y &Sus_TirPz, real_Y &Sus_Tirvz, real_Y &Tir_Pz, real_Y &Tir_vz, real_Y &Tir_rhoz, real_Y &Tir_Re);
	void pull_fm (const real_Y &Axl_Trq, const real_Y &Brk_Prs, const real_Y &Tir_Fx, const real_Y &Tir_My, const real_Y &Tir_Fz, const real_Y &Sus_Fz);
	void push_fm (real_Y &Brk_Trq);

	void push_drv (d_vec &derivatives);
private:
	//built in table
	const int m_truth_table[8] = {0,1,0,0,1,1,1,0};	//for lock logic
	//built in parameters
	const real_Y m_lpf_wc = 200.0 * pi;
	const real_Y m_lpf_init = 0.0;
	//configurable parameters
	real_Y m_unloaded_radius, m_IYY, m_mass, m_br, m_disk_abore, \
	m_num_pads, m_Rm, m_mu_kinetic, m_mu_static;
	//inputs
	real_Y m_Gnd_Pz = NaN;
	real_Y m_Axl_Trq = NaN;
	real_Y m_Brk_Prs = NaN;
	real_Y m_Tir_Fx = NaN;
	real_Y m_Tir_My = NaN;
	real_Y m_Sus_Fz = NaN;
	real_Y m_Tir_Fz = NaN;
	//continuous states
	real_Y m_unlocked_omega;
	real_Y m_Tir_Pz;
	real_Y m_Tir_vz;
	real_Y m_Sus_lpf_Fz = m_lpf_init;
	//continuous states derivative
	real_Y m_drv_unlocked_omega;
	real_Y m_drv_Tir_Pz;
	real_Y m_drv_Tir_vz;
	real_Y m_drv_Sus_lfp_Fz;
	//pre omega
	real_Y m_unlocked_omega_pre;
	//discrete states
	bool m_locked_flag;
	bool m_locked_state;
	//middle variables
	real_Y m_Tout = NaN;
	real_Y m_Tfmaxk = NaN;
	real_Y m_Tfmaxs = NaN;
	//outputs
	real_Y m_Tir_omega = NaN;
	real_Y m_Tir_Re = NaN;
	real_Y m_Sus_TirPz = NaN;
	real_Y m_Sus_Tirvz = NaN;
	real_Y m_Tir_rhoz = NaN;
	real_Y m_Tir_Brk_Trq = NaN;
};

}	//end of name space
#endif //WHEEL_DISKDRUM_TIRE_FIALA_HPP
